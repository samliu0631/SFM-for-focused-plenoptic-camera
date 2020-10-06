function ValidPtsID = SelectReconstructed3DPtsByPDDRPE(RelativeSturetureBag,InfoBag, AlgoConfig)
    % data prepare.
    maxPDDCenterError   = AlgoConfig.maxPDDCenterError_PtsRP;
    maxPDRError         = AlgoConfig.maxPDRError_PtsRP;
    
    matchedPDF    = RelativeSturetureBag.matchedPDF;
    Pts3D         = RelativeSturetureBag.Pts3D;    
    poses         = RelativeSturetureBag.poses;
    f1            = RelativeSturetureBag.f1;
    f2            = RelativeSturetureBag.f2;
    Rh            = poses{f1}(1:3, 1:3);  % 获得当前参考帧到世界坐标系的位姿变换矩阵
    th1           = poses{f1}(1:3, 4);
    extMat1       = [Rh' -Rh'*th1; zeros(1, 3) 1];
    Rh            = poses{f2}(1:3, 1:3);   % 获得当前对照帧到世界坐标系的位姿变换矩阵。
    th2           = poses{f2}(1:3, 4);
    extMat2       = [Rh' -Rh'*th2; zeros(1, 3) 1];
    
    % calculate the camera coorinates of 3D pts in each frames.
    NumPts            = size(Pts3D,1);
    PtsCameraCoords1  = extMat1*[Pts3D';ones(1,NumPts)];
    PtsCameraCoords2  = extMat2*[Pts3D';ones(1,NumPts)];
    PtsCameraCoords   = cell(2,1);
    PtsCameraCoords{1}=  PtsCameraCoords1(1:3,:);
    PtsCameraCoords{2}=  PtsCameraCoords2(1:3,:);
    
    % calculate the reprojection error for each 3D points.
    PDDCenterErrorsList  = zeros(NumPts,1);
    PDRErrorsList        = zeros(NumPts,1);
    for i = 1:size(matchedPDF,1)          % iterate two frames
        CurFramePDD     = matchedPDF{i};  % feature info for current frames.
        CurPtsCamCoords = PtsCameraCoords{i}; % Camera coordinates for all points.
        for j = 1:NumPts                      % go through all matched features.
            Cur_PDD = CurFramePDD(:,j)';
            if isempty( Cur_PDD)
                PDDCenterErrorsList(j) = PDDCenterErrorsList(j)+maxPDDCenterError*2+1;
                PDRErrorsList(j)       = PDRErrorsList(j)+maxPDRError*2+1;
                continue;
            end
            Mu                      = InfoBag.fx * CurPtsCamCoords(1, j)/CurPtsCamCoords(3, j) + InfoBag.cx;
            Mv                      = InfoBag.fy * CurPtsCamCoords(2, j)/CurPtsCamCoords(3, j) + InfoBag.cy;
            Rm                      = -InfoBag.K2 / CurPtsCamCoords(3, j) - InfoBag.K1;
            PDDCError               = sqrt( sum( ( Cur_PDD(1:2)-[Mu,Mv] ).^2 , 2  ) );
            PDRError                = abs(Rm-Cur_PDD (3));
            PDDCenterErrorsList(j)  = PDDCenterErrorsList(j)+PDDCError;
            PDRErrorsList(j)        = PDRErrorsList(j)+PDRError;
        end
    end
    PDDCenterErrorsList = PDDCenterErrorsList./size(matchedPDF,1); % Reprejection error for each reconstructed 3D point.
    PDRErrorsList       = PDRErrorsList./size(matchedPDF,1);
    
    % Validate the reconstructed point by angle.
    Vector1  = th1'- Pts3D;
    Vector2  = th2'- Pts3D;
    
    NormVector1 = sqrt( sum(Vector1.^2, 2) );
    NormVector2 = sqrt( sum(Vector2.^2, 2) );
    
    costheta = sum( Vector1.*Vector2 , 2 )./(NormVector1.*NormVector2);
    theta = acos(costheta)/pi*180;
    ValidAngleID = theta > AlgoConfig.maxAngle;
    
    % valide the distance between reconstructed 3D Point and camera.
    Pts2CamDist1  = PtsCameraCoords{1}(3,:);   
    Pts2CamDist2  = PtsCameraCoords{2}(3,:);  
    ValidDistID   =  ( Pts2CamDist1 >0 ) & ( Pts2CamDist1 < AlgoConfig.maxIncreTriangleDist ) & ( Pts2CamDist2 >0 ) & ( Pts2CamDist2 < AlgoConfig.maxIncreTriangleDist) ;
    ValidID  = (PDDCenterErrorsList < maxPDDCenterError) & (PDRErrorsList<maxPDRError) & ValidDistID'& ValidAngleID; 
    ValidPtsID = 1:NumPts;
    ValidPtsID  = ValidPtsID(ValidID);
end