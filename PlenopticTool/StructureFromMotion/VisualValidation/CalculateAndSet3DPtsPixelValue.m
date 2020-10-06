function StructureBag = CalculateAndSet3DPtsPixelValue(StructureBag, QueryImgInfo,PDFCell, TotalImgPath, TotalImgSpec)
    % Read the Total focus image.
    [FileList_TFI, BasePath_TFI]   = ReadRawImgInfo(TotalImgPath, TotalImgSpec);
    visMatrixRegister = StructureBag.visMatrix_est(StructureBag.ValidPtsID, QueryImgInfo.registered);
    visBoolRegister   = StructureBag.visBool_est(StructureBag.ValidPtsID, QueryImgInfo.registered);
    RegiFrameNum      = size(visMatrixRegister,2);
    ReconPtsNum       = size(visMatrixRegister,1);
    PixelValueList    = zeros(ReconPtsNum,1);
    MatchNumPPts      = sum(visBoolRegister,2);
    for i = 1: RegiFrameNum

        CurFrameID = QueryImgInfo.registered(i);
        CurPDF     = PDFCell{CurFrameID};

        % extract current frame feature id.
        FeatinFrameID = visMatrixRegister(:,i);

        % Get the valid feature index. 
        ValidIndex  = find(FeatinFrameID > 0);
        ValidFeatinFrameID = FeatinFrameID(ValidIndex);

        % Extract the feature location of current feature .
        FeatLocinTFI  = CurPDF(ValidFeatinFrameID,1:2)./2;
        FeatLocInt    = round(FeatLocinTFI);

        % Read the total focus image.   
        TotalFImg      = ReadRawImg( BasePath_TFI, FileList_TFI, CurFrameID);
        Height         = size( TotalFImg, 1 );
        Width          = size( TotalFImg, 2 );

        % Check the location 
        FeatLocInt( FeatLocInt(:,1) <= 0      , 1 ) = 0;
        FeatLocInt( FeatLocInt(:,1) >= Width  , 1 ) = Width;
        FeatLocInt( FeatLocInt(:,2) <= 0      , 2 ) = 0;
        FeatLocInt( FeatLocInt(:,2) >= Height , 2 ) = Height;
        FeatLinearLoc  = (FeatLocInt(:,1)-1).*Height + FeatLocInt(:,2);

        % Get the pixel value.
        PixelValue  = TotalFImg(FeatLinearLoc);

        %Store the Pixel value. 
        PixelValueList(ValidIndex) = PixelValueList(ValidIndex) + PixelValue;

    end
    Pts3DPixel  = PixelValueList./MatchNumPPts;
    StructureBag.Pts3DColor  = [Pts3DPixel,Pts3DPixel,Pts3DPixel];

end