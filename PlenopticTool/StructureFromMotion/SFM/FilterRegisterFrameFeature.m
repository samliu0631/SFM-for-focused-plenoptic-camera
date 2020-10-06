function StructureBag = FilterRegisterFrameFeature(StructureBag, QueryImgInfo , InfoBag, AlgoConfig)
    % FUNCTION : filter the false inter-match between registered frames and
    % current registered frame by updating the visMatrix and visBool.
    % Data input.
    visMatrix    = StructureBag.visMatrix_est;
    visBool      = StructureBag.visBool_est;
    Pts3D        = StructureBag.Pts3D;
    PtsID        = StructureBag.ValidPtsID;
    Poses        = StructureBag.poses;    
    nextFrame    = QueryImgInfo.nextFrame;
    Features_est = QueryImgInfo.Features_est;

    % Find points seen by current frame.
    SeenID     = find(visBool(PtsID, nextFrame));  % find the id of points seen by current frame within the reconstructed points.
    SeenPts3D  = Pts3D(SeenID,:);    % Extract the  location of reconstructed points.
    SeenPtsID  = PtsID(SeenID);      % The point id within  visMatrix.
    NumSeenPts = size(SeenPtsID,1);
    
    % Convert World coordinates to camera coordinates
    Rh         = Poses{nextFrame}(1:3,1:3);
    th         = Poses{nextFrame}(1:3,4);  
    extMat     = [Rh' -Rh'*th; zeros(1, 3) 1];
    WorldPts   = [SeenPts3D, ones(NumSeenPts,1)];
    CameraPts  = extMat*WorldPts';
    CurPtsCamCoords  = CameraPts(1:3,:);
    
    % Extract the location of seen feature for current frame.
    FeatureInfoCell = Features_est{nextFrame}( visMatrix( SeenPtsID, nextFrame ) );    
    
    % Calculate RPE.  
    ReproErrors = CalculateRPE(CurPtsCamCoords,FeatureInfoCell, InfoBag );
    
    % Update visMatrix and visBool.
    OutlierID  =  (ReproErrors > AlgoConfig.NextFramePtsRPE);
    OutPtsID   =  SeenPtsID(OutlierID);
    if ~isempty(OutPtsID)
        visMatrix(OutPtsID, nextFrame) = 0;
        visBool(OutPtsID, nextFrame) = 0;
    end
    
    % DataOutput 
    StructureBag.visMatrix_est  = visMatrix;
    StructureBag.visBool_est    = visBool;
end