function [paramFinal,FrameIdUsed]  = GetCalibResultsUsingDMTFI( CalibPath, CalibDM, CalibTFI ,CalibFileSpec,GridCoords,LensletGridModel,BoardCornerGap,BoardCornerXYNum)
    flag1 = exist([CalibPath,'/DetectResultsDMTFI.mat'],'file');
    if flag1==2
        load([CalibPath,'/DetectResultsDMTFI.mat'], 'allPts','allOmega');
    else
        [allPts,allOmega ] = DetectCornerFromDMTFI(CalibPath, CalibDM, CalibTFI ,CalibFileSpec,GridCoords,LensletGridModel);
        save([CalibPath,'/DetectResultsDMTFI.mat'], 'allPts','allOmega');
    end
    
    % Calibrate the plenoptic camera.
   [paramFinal,FrameIdUsed] = CalibPlenopticCameraUseCorner(allPts,allOmega,GridCoords,BoardCornerGap,BoardCornerXYNum);

end


function [allPts,allOmega ] = DetectCornerFromDMTFI(CalibPath, CalibDM, CalibTFI ,CalibFileSpec,GridCoords,LensletGridModel)
    % Read total focus image info.
    [FileList_TFI, BasePath_TFI]   = ReadRawImgInfo(CalibTFI, CalibFileSpec);    % Get image numbers and names from image lists.

    % Read Depth map info.
    [FileList_DM, BasePath_DM]     = ReadRawImgInfo(CalibDM, CalibFileSpec);    % Get image numbers and names from image lists.

    % Read Raw image info.
    %[FileList_Raw, BasePath_Raw]     = ReadRawImgInfo(CalibPath, CalibFileSpec);    % Get image numbers and names from image lists.

    % Start to work.
    ImageNum = length(FileList_TFI);
    allOmega = cell(ImageNum,1);
    allPts = cell(ImageNum,1);
    for id = 1:ImageNum

        % Read the total focus image .
        TotalFImg      = ReadRawImg( BasePath_TFI, FileList_TFI, id );  % Read the raw light field image.

        % Extract SIFT features;
        [cornerCenter,boardSize] = detectCheckerboardPoints(TotalFImg);

        if isempty(cornerCenter)
           continue;
        end 
        % Read the depth maps
        DepthMap  = ReadRawImg( BasePath_DM, FileList_DM, id );  % Read the raw light field image.

        % Convert the gray value to virtual depth.
        DepthMap  = 1./(1-DepthMap);

        % Convert the Feature coordinates from TFI to DM.
        Width_TFI    = size(TotalFImg,2);
        Width_DM     = size(DepthMap,2);
        Ratio        = Width_DM/Width_TFI;
        FeatLocIntDM = round(cornerCenter.*Ratio);

        % Extract the depth info from depth maps.
        Height_DM   = size(DepthMap,1);
        FeatID       = ( FeatLocIntDM(:,1)-1 ).*Height_DM + FeatLocIntDM(:,2);
        lfFeatureVD  = -DepthMap(FeatID);

        % The calculated results are actually the pdf.
        TFRawRatio = 2;
        PDFCurrent = [ cornerCenter.*TFRawRatio,lfFeatureVD];


        %Visual Validation.
        FeatNum = size(cornerCenter,1);
        allPtsPF = cell(FeatNum,1);
        allOmegaPF = cell(FeatNum,1);
        LensRadius = LensletGridModel.HSpacing/2;
        %RawImg     = ReadRawImg( BasePath_Raw, FileList_Raw, id );  % Read the raw light field image.
        %figure;imshow(RawImg);hold on;
        for i = 1 : FeatNum
            allProjPtsPerCorner      = ProjectPDF2Img(PDFCurrent(i,:), GridCoords, LensRadius,-1 ,0);
        %    plot(allProjPtsPerCorner(:,1),allProjPtsPerCorner(:,2),'*');
            allPtsPF{i} = allProjPtsPerCorner;
            allOmegaPF{i} = PDFCurrent(i,:);
        end
        %hold off;

        % Store the PDF
        allOmega{id} = allOmegaPF;
        allPts{id}  = allPtsPF;

    end

end
