function  [StructureBag,RegisterFlag ] = RemoveOutliersByPDDRPE(NextFramePoseBag, StructureBag, InfoBag, AlgoConfig)
    % Prepare data.  
    maxPDDCenterError   = AlgoConfig.maxPDDCenterError;
    maxPDRError         = AlgoConfig.maxPDRError;
    
    pts3dIds           = StructureBag.ValidPtsID;
    poses              = StructureBag.poses;
    Pts3D              = StructureBag.Pts3D;
    nextFrame          = NextFramePoseBag.nextFrame;
    Seen3dReconsPtsID  = NextFramePoseBag.Seen3dReconsPtsID;  
    FeatIDinFrame      = NextFramePoseBag.FeatIDinFrame;

    
    % calculate the camera coorinates of 3D pts in each frames.
    Rh               = poses{nextFrame}(1:3, 1:3);  % 获得当前参考帧到世界坐标系的位姿变换矩阵
    th               = poses{nextFrame}(1:3, 4);
    extMat1          = [Rh' -Rh'*th; zeros(1, 3) 1];    
    NumPts           = size(Pts3D,1);
    PtsCameraCoords  = extMat1*[Pts3D';ones(1,NumPts)];
    PtsCameraCoords  =  PtsCameraCoords(1:3,:);
    
    % calculate the reprojection error for each 3D points.
    NumSeenPts          = size(Seen3dReconsPtsID,1);
    PDDCenterErrorsList = zeros(NumSeenPts,1);
    PDRErrorList        = zeros(NumSeenPts,1);
    CurPtsCamCoords     = PtsCameraCoords(:,Seen3dReconsPtsID); % Camera coordinates for all points.
    for j = 1:NumSeenPts                      % go through all matched features.
        CurPDD              = NextFramePoseBag.PDF(:,FeatIDinFrame(j))';
        if isempty(CurPDD)|| ( abs(CurPDD(3))< AlgoConfig.minValidPDR )|| (abs(CurPDD(3))> AlgoConfig.maxValidPDR)
            PDDCenterErrorsList(j) = maxPDDCenterError+1;  % If there is no projected points of current PDF, then the PDFR is too small to garantee good matching results.
            PDRErrorList(j)        = maxPDRError +1;
            continue;
        end
        Mu                      = InfoBag.fx * CurPtsCamCoords(1, j)/CurPtsCamCoords(3, j) + InfoBag.cx;
        Mv                      = InfoBag.fy * CurPtsCamCoords(2, j)/CurPtsCamCoords(3, j) + InfoBag.cy;
        Rm                      = -InfoBag.K2 / CurPtsCamCoords(3, j) - InfoBag.K1;
        PDDCError               = sqrt( sum( ( CurPDD (1:2)-[Mu,Mv] ).^2 , 2  ) );
        PDRError                = abs(Rm-CurPDD (3));
        PDDCenterErrorsList(j)  = PDDCError;  % If there is no projected points of current PDF, then the PDFR is too small to garantee good matching results.
        PDRErrorList(j)         = PDRError;
    end
    
    
    % Calculate Outliers.
    OutlierID           = (PDDCenterErrorsList > maxPDDCenterError) | ( PDRErrorList > maxPDRError );
    Ourlier3DPtsID      = Seen3dReconsPtsID(OutlierID);
    
    % Check the inner 3D-2D match number.
    OutNum   = size(Ourlier3DPtsID,1);
    InnerNum = NumSeenPts-OutNum;
    RegisterFlag = 1;
    if  InnerNum < AlgoConfig.Min3D2DMatchNum
        RegisterFlag = 0;
        StructureBag.poses{nextFrame} = [];    
        return;
    end    
    
    % Update the visMatrix and visBool
    StructureBag.visMatrix_est(pts3dIds(Ourlier3DPtsID),nextFrame) = 0;
    StructureBag.visBool_est(pts3dIds(Ourlier3DPtsID),nextFrame)   = 0;
    
    
    
    
end