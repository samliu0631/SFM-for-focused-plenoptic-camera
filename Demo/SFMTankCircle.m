clc,clear
warning('off');
MainPath             = 'F:\博士课题\GitFlowSpace\Data' ;    
RawDataPath          = [MainPath,'\FusionData\Tank\TankCircle\RawProc'];
DepthPath            = [MainPath,'\FusionData\Tank\TankCircle\Depth'];  
TotalImgPath         = [MainPath,'\FusionData\Tank\TankCircle\TotalImg'];
CalibRawPath         = [MainPath,'\FusionData\Tank\Calib\RawProc'];
CalibDM              = [MainPath,'\FusionData\Tank\Calib\Depth'];
CalibTFI             = [MainPath,'\FusionData\Tank\Calib\TotalImg'];
WhitePath            = [MainPath,'\FusionData\Tank\Calib\White'];
XMLPath              = [MainPath,'\FusionData\Tank\Calib\XML'];
FileSpecWhite        = '*.png';
DefaultFileSpec_Raw  = '*.png';
CalibFileSpec        = '*.png';
DepthSpec            = '*.png';
TotalImgSpec         = '*.png';
RoughRadius          = 32;
BoardCornerGap       = [15,15];
BoardCornerXYNum     = [14,19];  %  标定板的固有参数。先纵后横
MinPDR               = 2;  % minimum PDR.
MaxPDR               = 30;   % maximum PDR.

% Get the central position of micro images from XML file.
[LensletGridModel,GridCoords,ImgSize] = GetLensInfofromXML(WhitePath, XMLPath, FileSpecWhite);

% Calibration using the total focus image and depth maps. 
flag1 = exist([CalibRawPath,'/OurCalibResultonDMTFI.mat'],'file');
if flag1 == 2
    load([CalibRawPath,'/OurCalibResultonDMTFI.mat'], 'paramFinal','FrameIdUsed');
else
    [paramFinal,FrameIdUsed] = GetCalibResultsUsingDMTFI( CalibRawPath, CalibDM, CalibTFI ,CalibFileSpec,GridCoords,LensletGridModel,BoardCornerGap,BoardCornerXYNum);
    save([CalibRawPath,'/OurCalibResultonDMTFI.mat'], 'paramFinal','FrameIdUsed');
end

% Prepare Camera InfoBag.
InfoBag = SetInfoBag(paramFinal,LensletGridModel, GridCoords,ImgSize);

% Extract feature from Depth maps and total focus image.
flag2 = exist([RawDataPath,'\FeatureDectionResultOnTotalFocusImg.mat'],'file');
if flag2 == 2
    load([RawDataPath,'\FeatureDectionResultOnTotalFocusImg.mat'], 'PDFCell' , 'DescCell','FeaturesCell' );
else  
    [PDFCell,DescCell] = DetectPDFfromTotalFocusImgDepthMap( DepthPath, TotalImgPath, DepthSpec, TotalImgSpec,ImgSize, GridCoords);
    % Check the PDR of the input data. PDR should be restriained to valid range.
    [PDFCell, DescCell] = CheckandFilterInputDatabyPDR(PDFCell, DescCell,MinPDR,MaxPDR);    
    FeaturesCell        = ProjectPDF2RawImg(PDFCell, InfoBag.LensletGridModel, InfoBag.GridCoords);    
    save([RawDataPath,'\FeatureDectionResultOnTotalFocusImg.mat'], 'PDFCell' , 'DescCell', 'FeaturesCell' );
end

% Visual validation of the detected results.
%VisualValidateAllSIFTFeature(PDFCell,RawDataPath, DefaultFileSpec_Raw,LensletGridModel, GridCoords);

% Prepare data for the UFPSFM.
flag2 = exist([RawDataPath,'/SFMalldata.mat'],'file');
if flag2 == 2
    load([RawDataPath,'/SFMalldata.mat'], 'GeneratedDataBag' , 'InfoBag' );
else
    GeneratedDataBag     = SetDataBag(InfoBag, PDFCell, DescCell, FeaturesCell);
    save([RawDataPath,'/SFMalldata.mat'], 'GeneratedDataBag' , 'InfoBag' );
end

% manually set the initial image id. % This part can be removed.
GeneratedDataBag.f1   = 1;
GeneratedDataBag.f2   = 2;

% SFM PART
flag2 = exist([RawDataPath,'/SFMdata.mat'],'file');
if flag2==2
    load([RawDataPath,'/SFMdata.mat'], 'StructureBag', 'QueryImgInfo', 'InfoBag' );
else
    [StructureBag, QueryImgInfo, InfoBag] = UPFSFM(GeneratedDataBag,InfoBag);
    % Global SBA.
    fprintf('Registered %d new images, calling global Bundle Adjustment...\n', numel(QueryImgInfo.registered));
    StructureBag     = LFSBA(StructureBag, QueryImgInfo,InfoBag);
    save([RawDataPath,'/SFMdata.mat'],  'StructureBag', 'QueryImgInfo', 'InfoBag' );
end

% Visual Confirmation。
VisualConfirm(InfoBag,StructureBag);

flag2 = exist([RawDataPath,'/ColorStructureData.mat'],'file');
if flag2==2
    load([RawDataPath,'/ColorStructureData.mat'], 'StructureBag');
else
    % Get the 3D Pts Pixel Value.
    StructureBag = CalculateAndSet3DPtsPixelValue(StructureBag, QueryImgInfo,PDFCell, TotalImgPath, TotalImgSpec);
    save([RawDataPath,'/ColorStructureData.mat'], 'StructureBag');
end


VisualConfirm(InfoBag,StructureBag);

% Check if there is pixel value.
if isfield(StructureBag, 'Pts3DColor')
    MyPointcloud  = pointCloud(StructureBag.Pts3D,'Color',StructureBag.Pts3DColor);
else
    MyPointcloud  = pointCloud(StructureBag.Pts3D);
end
pcwrite(MyPointcloud,'TankCircle','PLYFormat','binary');



