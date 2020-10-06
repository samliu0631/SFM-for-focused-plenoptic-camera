function LocSubCamera = CalculateVCamLocation(InfoBag,LensID)
    K1 = InfoBag.K1;
    K2 = InfoBag.K2;
    fx = InfoBag.fx;
    fy = InfoBag.fy;
    GridCoords = InfoBag.GridCoords ;
    GridCoordsX = GridCoords(:,:,1);
    GridCoordsY = GridCoords(:,:,2);
    McImgCtCrd  = [GridCoordsX(LensID),GridCoordsY(LensID)];
    cx = InfoBag.cx;
    cy = InfoBag.cy;
    UVc = McImgCtCrd-[cx,cy];
    LocSubCamera = [-K2*UVc(1)/(K1*fx),  -K2*UVc(2)/(K1*fy),    -K2/K1]';

end