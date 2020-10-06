function RPE = CalculateRPEofPDF(CurPtsCamCoords,PDFMeasured, InfoBag)
    fx = InfoBag.fx;
    fy = InfoBag.fy;
    K1 = InfoBag.K1;
    K2 = InfoBag.K2;
    cu = InfoBag.cx;
    cv = InfoBag.cy;
    Mu = fx*(  CurPtsCamCoords(1)/CurPtsCamCoords(3)  )+cu;
    Mv = fy*(  CurPtsCamCoords(2)/CurPtsCamCoords(3)  )+cv;
    RPE = sqrt( sum( (PDFMeasured(1:2)'-[Mu,Mv]).^2 ) );
end