function InfoBag = SetInfoBag(paramFinal,LensletGridModel, GridCoords,ImgSize)
    InfoBag.K1 = paramFinal(1);
    InfoBag.K2 = paramFinal(2);
    InfoBag.fx = paramFinal(3);
    InfoBag.fy = paramFinal(4);
    InfoBag.cx = paramFinal(5);
    InfoBag.cy = paramFinal(6);
    InfoBag.k1 = paramFinal(7);
    InfoBag.k2 = paramFinal(8);
    InfoBag.pixelX   = ImgSize(2);
    InfoBag.pixelY   = ImgSize(1);
    InfoBag.LensletGridModel  = LensletGridModel;
    InfoBag.GridCoords        = GridCoords;
end