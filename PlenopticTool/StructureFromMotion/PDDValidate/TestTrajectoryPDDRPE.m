function OutlierFlag = TestTrajectoryPDDRPE(FrameInfoBag,InfoBag, AlgoConfig)

    Pts3D          = FrameInfoBag.CurPts;
    PDD            = FrameInfoBag.PDD;
    FramePose      = FrameInfoBag.CurPoseMatrix;
    
    % Get the rotation matrix
    Rh = FramePose(1:3, 1:3);
    th = FramePose(1:3, 4);    
    extMat1 = [Rh' -Rh'*th; zeros(1, 3) 1];
    
    % Calculate the 3D Point in current camera coordinates.
    PtsCameraCoords = extMat1*[Pts3D';1];
    
    % calculate the PDD RPE each 3D points.
    CurPtsCamCoords = PtsCameraCoords(1:3,:); % Camera coordinates for all points.    
    Cur_PDD         = PDD';
    if isempty(Cur_PDD)
        OutlierFlag = 1;
        return;
    end
    Mu                      = InfoBag.fx * CurPtsCamCoords(1)/CurPtsCamCoords(3) + InfoBag.cx;
    Mv                      = InfoBag.fy * CurPtsCamCoords(2)/CurPtsCamCoords(3) + InfoBag.cy;
    Rm                      = -InfoBag.K2 / CurPtsCamCoords(3) - InfoBag.K1;
    PDDCError               = sqrt( sum( ( Cur_PDD(1:2)-[Mu,Mv] ).^2 , 2  ) );
    PDRError                = abs(Rm-Cur_PDD (3));

    if  (PDDCError > AlgoConfig.maxPDDCenterError_PtsRP ) || (PDRError > AlgoConfig.maxPDRError_PtsRP )  
        OutlierFlag = 1;
    else
        OutlierFlag = 0;
    end
   


end
