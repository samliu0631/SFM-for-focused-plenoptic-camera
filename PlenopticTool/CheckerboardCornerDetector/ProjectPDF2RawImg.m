function FeaturesCell = ProjectPDF2RawImg(PDFCell, LensletGridModel, GridCoords)
    FrameNum     = size(PDFCell,1);    
    MicImgRadius  = LensletGridModel.HSpacing/2;
    FeaturesCell = cell(FrameNum,1);    
    for i = 1 : FrameNum
        PDFCollector = PDFCell{i};
        pdfNum       = size(PDFCollector,1);
        FeatCellPF   = cell(pdfNum,1);
        fprintf('Generate the 2D feature of Frame %d\n', i);
        for j = 1:pdfNum
        % parfor j = 1:pdfNum  % If there is multi-core cpu, use this to
        % replace for j = 1:pdfNum.
            CurPDF = PDFCollector(j,:);
            ProjPtsPerCorner   = ProjectPDF2Img(CurPDF, GridCoords, MicImgRadius,-1 ,0); % 这个需要后期测试。
            FeatCellPF{j}      = ProjPtsPerCorner;
        end
        FeaturesCell{i}  = FeatCellPF;        
    end
end