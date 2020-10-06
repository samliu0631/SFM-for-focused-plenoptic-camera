function VCamExtrinsic = CalcualteSubCamExtrinsicMatrix(InfoBag)
    % Calculate the Extrinsic for all sub-camera.
    K1           = InfoBag.K1;
    K2           = InfoBag.K2;
    fx           = InfoBag.fx;
    fy           = InfoBag.fy;
    GridCoords   = InfoBag.GridCoords ;
    GridCoordsX  = GridCoords(:,:,1);
    GridCoordsY  = GridCoords(:,:,2);
    McImgCtCrd   = [GridCoordsX(:),GridCoordsY(:)];
    cx           = InfoBag.cx;
    cy           = InfoBag.cy;
    Ucv          = McImgCtCrd-[cx,cy];
    SubCamSize   = numel(GridCoordsX);
    VCamExtrinsic= zeros(SubCamSize,7);
    CsubCam      = -[K2*Ucv(:,1)./(K1*fx),K2.*Ucv(:,2)./(K1*fy), K2/K1.*ones(SubCamSize,1) ];
    VCamExtrinsic(:,1) = 1;
    VCamExtrinsic(:,5:7) = -CsubCam;
end
