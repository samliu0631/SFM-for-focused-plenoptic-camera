function Kv = CalculateVCamMatrix(InfoBag,LensID)

    LensletGridModel  = InfoBag.LensletGridModel;
    Rmi   = LensletGridModel.HSpacing/2;
    K1 = InfoBag.K1;
    fx = InfoBag.fx;
    fy = InfoBag.fy;
    GridCoords = InfoBag.GridCoords ;
    GridCoordsX = GridCoords(:,:,1);
    GridCoordsY = GridCoords(:,:,2);
    McImgCtCrd  = [GridCoordsX(LensID),GridCoordsY(LensID)];
    cx = InfoBag.cx;
    cy = InfoBag.cy;
    Ucv = McImgCtCrd-[cx,cy];
    Kv = [fx/K1,   0,     -Ucv(1)/K1+Rmi;
          0    ,   fy/K1, -Ucv(2)/K1+Rmi;
          0    ,   0,         1     ];
end