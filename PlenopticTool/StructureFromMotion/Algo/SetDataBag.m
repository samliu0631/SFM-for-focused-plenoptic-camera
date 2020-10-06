function GeneratedDataBag = SetDataBag(InfoBag, PDFCell, DescCell ,FeaturesCell)
    InterMatchConfig.matchThresh    = 1.5;
    InterMatchConfig.essHomRatio    = 3;
    InterMatchConfig.filterMatches  = true;
    InterMatchConfig.K              = [InfoBag.fx,   0,   InfoBag.cx;    0,   InfoBag.fy   ,InfoBag.cy;   0,0,1];
    if isfield(InfoBag,'SFMWindowSize')
        InterMatchConfig.SFMWindowSize  = InfoBag.SFMWindowSize;
    end
    tic;
    [matches, geomVer]              = MatchAcrossImages( PDFCell, DescCell, InterMatchConfig);   
    toc;
    %save('MatchAndGeoVer.mat', 'matches', 'geomVer');
     
    [visMatrix, visBool]            = getVisibilityMatrix(matches);
    
    [numMatches, visBool]           = getPairwiseNumMatches(matches, visBool); 
    [f1, f2]                        = selectInitialFrames(geomVer, numMatches);  % 选择初始的图像对。
    
    GeneratedDataBag.Features_est   = FeaturesCell;
    GeneratedDataBag.visBool_est    = visBool;
    GeneratedDataBag.visMatrix_est  = visMatrix;
    GeneratedDataBag.geomVer_est    = geomVer;
    PDFCellT                        = transposePDFCell(PDFCell);
    GeneratedDataBag.PDFCell        = PDFCellT; % 要把内容转职一下。      
    GeneratedDataBag.f1             = f1; 
    GeneratedDataBag.f2             = f2; 
end


function PDFCellT = transposePDFCell(PDFCell)
    FrameNum  = size(PDFCell,1);
    PDFCellT  = cell(FrameNum,1); 
    for i = 1:FrameNum
        CurPDF      = PDFCell{i};
        PDFCellT{i} = CurPDF';        
    end
end








