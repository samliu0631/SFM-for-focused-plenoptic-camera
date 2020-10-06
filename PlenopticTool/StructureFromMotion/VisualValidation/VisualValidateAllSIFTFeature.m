function VisualValidateAllSIFTFeature(PDFCell,DataPath, DefaultFileSpec_Raw,LensletGridModel, GridCoords)
    FrameNum  = size(PDFCell,1);
    [FileList_raw, BasePath_raw]   = ReadRawImgInfo(DataPath, DefaultFileSpec_Raw);
    for id = 1 :FrameNum
        Img_raw       = ReadRawImg( BasePath_raw, FileList_raw, id );
        PDFCollector  = PDFCell{id};
        VisualValidatePDF(PDFCollector, LensletGridModel, GridCoords, Img_raw);
    end
end