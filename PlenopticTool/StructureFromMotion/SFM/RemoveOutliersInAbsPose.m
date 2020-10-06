function StructureBag = RemoveOutliersInAbsPose(NextFramePoseBag, StructureBag, InfoBag, AlgoConfig)
    % Prepare data.  
    pts3dIds           = StructureBag.ValidPtsID;
    poses              = StructureBag.poses;
    Pts3D              = StructureBag.Pts3D;
    nextFrame          = NextFramePoseBag.nextFrame;
    FeatureCoordsCell  = NextFramePoseBag.FeatureCoordsCell;
    Seen3dReconsPtsID  = NextFramePoseBag.Seen3dReconsPtsID;  
    FeatIDinFrame      = NextFramePoseBag.FeatIDinFrame;
    GridCoords         = InfoBag.GridCoords;
    GridCoordsX        = GridCoords(:,:,1);
    GridCoordsY        = GridCoords(:,:,2);
    
    % calculate the camera coorinates of 3D pts in each frames.
    Rh              = poses{nextFrame}(1:3, 1:3);  % 获得当前参考帧到世界坐标系的位姿变换矩阵
    th              = poses{nextFrame}(1:3, 4);
    extMat1         = [Rh' -Rh'*th; zeros(1, 3) 1];    
    NumPts          = size(Pts3D,1);
    PtsCameraCoords = extMat1*[Pts3D';ones(1,NumPts)];
    PtsCameraCoords =  PtsCameraCoords(1:3,:);
    
    % calculate the reprojection error for each 3D points.
    NumSeenPts      = size(Seen3dReconsPtsID,1);
    ReproErrors     = zeros(NumSeenPts,1);
    FeatureInfoCell = FeatureCoordsCell;  % feature info for current frames.
    CurPtsCamCoords = PtsCameraCoords(:,Seen3dReconsPtsID); % Camera coordinates for all points.
    for j = 1:NumSeenPts                      % go through all matched features.
        Cur_FeatureInfo = FeatureInfoCell{FeatIDinFrame(j)};  % make sure the 2D feature correspond with the 3D feature.
        if isempty(Cur_FeatureInfo)
            ReproErrors(j)= AlgoConfig.AbsPoseMaxRPE;  % If there is no projected points of current PDF, then the PDFR is too small to garantee good matching results.
            continue;
        end
        Cur_PtsLensID = Cur_FeatureInfo(:,3);
        ucs           = GridCoordsX(Cur_PtsLensID);                                             % 微透镜中心X像素坐标。
        vcs           = GridCoordsY(Cur_PtsLensID);
        du            = InfoBag.fx * CurPtsCamCoords(1, j) - CurPtsCamCoords(3, j).*(ucs - InfoBag.cx);
        dv            = InfoBag.fy * CurPtsCamCoords(2, j) - CurPtsCamCoords(3, j).*(vcs - InfoBag.cy);
        nominator     = 1./(InfoBag.K1 * CurPtsCamCoords(3, j) + InfoBag.K2);
        du            = nominator.*du;
        dv            = nominator.*dv;
        us            = ucs+du;
        vs            = vcs+dv;
        err           = [us,vs]-Cur_FeatureInfo(:,1:2);   % calculate reprojection error.
        err           = mean( sqrt( sum( err.^2 , 2 ) ) );
        ReproErrors(j)= err;
    end
    ValidPtsID = find(ReproErrors < AlgoConfig.AbsPoseMaxRPE );   % id of validated reconstructed 3d points.  
    
    % update the visMatrix and visBool.
    Ourlier3DPtsID = Seen3dReconsPtsID( ~ismember( [1:NumSeenPts], ValidPtsID) );   
    StructureBag.visMatrix_est(pts3dIds(Ourlier3DPtsID),nextFrame) = 0;
    StructureBag.visBool_est(pts3dIds(Ourlier3DPtsID),nextFrame) = 0;
    
end