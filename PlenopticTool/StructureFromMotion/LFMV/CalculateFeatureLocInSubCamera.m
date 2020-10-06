function FeatureRelativeLoc = CalculateFeatureLocInSubCamera(InfoBag,LensID,FeatureCoords)
    LensletGridModel  = InfoBag.LensletGridModel;
    Rmi         = LensletGridModel.HSpacing/2;
    GridCoords  = InfoBag.GridCoords;
    GridCoordsX = GridCoords(:,:,1);
    GridCoordsY = GridCoords(:,:,2);
    McImgCtCrd  = [GridCoordsX(LensID),GridCoordsY(LensID)];
    FeatNum     = size(McImgCtCrd,1);
    FeatureRelativeLoc = FeatureCoords-McImgCtCrd + Rmi.*ones(FeatNum,2);
end