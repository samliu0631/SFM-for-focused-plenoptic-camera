function FeatLocInVCam = ConvertTheFeatLoc2VCam(FeaturesLoc, InfoBag)
    FrameNum    = size(FeaturesLoc,1);
    GridCoords  = InfoBag.GridCoords;
    sigma       = InfoBag.sigma;
    K1          = InfoBag.K1;
    cx          = InfoBag.cx;
    cy          = InfoBag.cy;
    GridCoordsX = GridCoords(:,:,1);
    GridCoordsY = GridCoords(:,:,2);    
    FeatLocInVCam = cell(FrameNum,1);
    
    for i = 1: FrameNum   % iterate all frames.
        CurFeatCell = FeaturesLoc{i};        
        FeatNum     = size(CurFeatCell,1);
        NewFeatCell = cell(FeatNum,1);
        for j = 1:FeatNum
            CurFeatInfo  = CurFeatCell{j};
            CurLensID    = CurFeatInfo(:,3);
            CurFeatLoc   = CurFeatInfo(:,1:2);            
            MicCenter    = [GridCoordsX(CurLensID), GridCoordsY(CurLensID)];
            UVc          = MicCenter-[cx,cy];
            DELTAFeatLoc = CurFeatLoc-MicCenter;
            offset       = UVc./K1+sigma;
            deltaUV      = DELTAFeatLoc+offset;    
            if sum(sum(deltaUV<0))
               fprintf('There is problem for the set of sigma.\n');
            end
            NewFeatCell{j} = [deltaUV,CurLensID];            
        end  
        FeatLocInVCam{i} = NewFeatCell;
    end
end