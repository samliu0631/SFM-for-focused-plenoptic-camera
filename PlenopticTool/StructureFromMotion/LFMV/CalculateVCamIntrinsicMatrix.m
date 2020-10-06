function InfoBag = CalculateVCamIntrinsicMatrix(InfoBag)
    % Calculate the intrinsic matrix of sub-camera.
    K1           = InfoBag.K1;
    fx           = InfoBag.fx;
    fy           = InfoBag.fy;
    MicImgRadius = InfoBag.Dmi/2;
    GridCoords   = InfoBag.GridCoords ;
    GridCoordsX  = GridCoords(:,:,1);
    GridCoordsY  = GridCoords(:,:,2);
    McImgCtCrd   = [GridCoordsX(:),GridCoordsY(:)];
    cx           = InfoBag.cx;
    cy           = InfoBag.cy;
    Ucv          = McImgCtCrd-[cx,cy];
    Dist         = sqrt( sum( Ucv.^2, 2 ) );
    MaxDist      = max(Dist);
    sigma        = ceil(MaxDist/K1+MicImgRadius+1);    
    Kintrinsic   = [fx/K1  ,   0,     sigma;
                    0    ,   fy/K1, sigma;
                    0    ,   0,         1];
    InfoBag.Kintrinsic  = Kintrinsic;
    InfoBag.sigma = sigma;
end
