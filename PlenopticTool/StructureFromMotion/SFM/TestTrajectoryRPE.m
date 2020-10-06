function OutlierFlag = TestTrajectoryRPE(FrameInfoBag,InfoBag, AlgoConfig)

    Pts3D          = FrameInfoBag.CurPts;
    FeatureArray   = FrameInfoBag.FeatureArray;
    FramePose      = FrameInfoBag.CurPoseMatrix;
    
    GridCoords     = InfoBag.GridCoords;
    GridCoordsX    = GridCoords(:,:,1);
    GridCoordsY    = GridCoords(:,:,2);
    
    Rh = FramePose(1:3, 1:3);
    th = FramePose(1:3, 4);    
    extMat1 = [Rh' -Rh'*th; zeros(1, 3) 1];
    
    
    PtsCameraCoords = extMat1*[Pts3D';1];
    
    % calculate the reprojection error for each 3D points.
    CurPtsCamCoords = PtsCameraCoords(1:3,:); % Camera coordinates for all points.    
    Cur_FeatureInfo = FeatureArray;
    if isempty(Cur_FeatureInfo)
        OutlierFlag = 1;
        return;
    end
    Cur_PtsLensID = Cur_FeatureInfo(:,3);
    ucs           = GridCoordsX(Cur_PtsLensID);                                             % Î¢Í¸¾µÖÐÐÄXÏñËØ×ø±ê¡£
    vcs           = GridCoordsY(Cur_PtsLensID);
    du            = InfoBag.fx * CurPtsCamCoords(1, 1) - CurPtsCamCoords(3, 1).*(ucs - InfoBag.cx);
    dv            = InfoBag.fy * CurPtsCamCoords(2, 1) - CurPtsCamCoords(3, 1).*(vcs - InfoBag.cy);
    nominator     = 1./(InfoBag.K1 * CurPtsCamCoords(3, 1) + InfoBag.K2);
    du            = nominator.*du;
    dv            = nominator.*dv;
    us            = ucs+du;
    vs            = vcs+dv;
    err           = [us,vs]-Cur_FeatureInfo(:,1:2);   % calculate reprojection error.
    err           = mean( sqrt( sum( err.^2 , 2 ) ) );
    ReproErrors   = err;

    if ReproErrors > AlgoConfig.TrajTestRPE
        OutlierFlag = 1;
    else
        OutlierFlag = 0;
    end
   


end
