function [PDFCell,DescCell] = DetectPDFfromTotalFocusImgDepthMap(DepthPath, TotalImgPath, DepthSpec, TotalImgSpec,ImgSize,GridCoords,varargin)
    if nargin > 6
        peakThresh = varargin{1};
    else
        peakThresh   = 0.006;
    end
    
        
    % Read total focus image info.
    [FileList_TFI, BasePath_TFI]   = ReadRawImgInfo(TotalImgPath, TotalImgSpec);    % Get image numbers and names from image lists.
    
    % Read Depth map info.
    [FileList_DM, BasePath_DM]     = ReadRawImgInfo(DepthPath, DepthSpec);    % Get image numbers and names from image lists.
    
    % Start to work.
    ImageNum = length(FileList_TFI);
    PDFCell  = cell(ImageNum,1);
    DescCell = cell(ImageNum,1);
    for id = 1:ImageNum 
        
        % Read the total focus image .
        TotalFImg      = ReadRawImg( BasePath_TFI, FileList_TFI, id );  % Read the raw light field image.        
        
        % Extract SIFT features;
        [lfFeatures, lfDescriptors]   = vl_sift(im2single(TotalFImg),'PeakThresh', peakThresh);
        
        % Read the depth maps
        DepthMap  = ReadRawImg( BasePath_DM, FileList_DM, id );  % Read the raw light field image.
        
        % Convert the gray value to virtual depth.
        DepthMap  = 1./(1-DepthMap);
        
        % Convert the Feature coordinates from TFI to DM.            
        Width_TFI    = size(TotalFImg,2);
        Width_DM     = size(DepthMap,2);
        Ratio        = Width_DM/Width_TFI;
        FeatLocIntDM = round(lfFeatures(1:2,:).*Ratio)';
        
        % Extract the depth info from depth maps. 
        Height_DM   = size(DepthMap,1);      
        FeatID       = ( FeatLocIntDM(:,1)-1 ).*Height_DM + FeatLocIntDM(:,2);
        lfFeatureVD  = -DepthMap(FeatID);            
                
        % The calculated results are actually the pdf.
        TFRawRatio = ImgSize(2)/Width_TFI;
        PDFCurrent = [ lfFeatures(1:2,:)'.*TFRawRatio,lfFeatureVD];
        
        
%         %Visual Validation.
%         RawImg = imread('E:\Sam\FusionData\OutDoors\OutDoors100\RawProc\Image_RawProc_1151.png');
%         LensRadius = 16;
%         figure;imshow(RawImg);
%         FeatNum = size(lfFeatures,2);
%         hold on; 
%         for i = 1 : FeatNum
%             allProjPtsPerCorner      = ProjectPDF2Img(PDFCurrent(i,:), GridCoords, LensRadius,-1 ,0);
%             plot(allProjPtsPerCorner(:,1),allProjPtsPerCorner(:,2),'.', 'MarkerSize',5); %
%         end 
%         hold off;
        
        % Store the PDF
        PDFCell{id} = PDFCurrent;
        DescCell{id} =  lfDescriptors';
        if mod(id,10)==0
            fprintf('Finish detecting %d frames\n',id);    
        end
    end
end