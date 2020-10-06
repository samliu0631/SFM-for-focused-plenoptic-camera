% Function: Reconstruct 3D points from the matching results between the
% nextframe and current registered frame.

function  StructureBag = incrementalReconstruct( StructureBag, QueryImgInfo, InfoBag, AlgoConfig )
    % Data Input.
    pts3dIds           = StructureBag.ValidPtsID;    % the reconstructed points id so far.
    regId              = QueryImgInfo.RegFrameID;    % the registered frame id in the img pair.
    nextFrame          = QueryImgInfo.nextFrame;     % the next frame id in the img pair. 
    PDFCell            = QueryImgInfo.PDFCell;       % the PDF of all frames.
    Features_est       = QueryImgInfo.Features_est;  % the feature location of all frames.
    
    % Extract the Pts ID of newly matched points.
    matchPtsIds        = StructureBag.visBool_est(:, regId) .* StructureBag.visBool_est(:, nextFrame);  
    matchPtsIds        = find(matchPtsIds);                             % The matched 3d point id of current img pair¡£
    newPtsIds          = matchPtsIds(~ismember(matchPtsIds, pts3dIds)); % Extract the new 3d point id.
    
    % Check if there is new point.
    if (isempty(newPtsIds))  
        return;
    end
    
    % Validate the matched feature by PDD RPE.
    [StructureBag , ValidMatchedindex] = ValidateIncrementalMatchedFeaturePairbyPDDRPE(StructureBag,regId,nextFrame,InfoBag,PDFCell,newPtsIds,AlgoConfig);
    newPtsIds          = newPtsIds(ValidMatchedindex);   
       
    
    % Extract validated matched feature info.
    matchedFeatInfo      = cell(2,1);
    matchedPDF           = cell(2,1);
    FeatIDinFrameReg     = StructureBag.visMatrix_est(newPtsIds, regId);
    FeatIDinFrameNext    = StructureBag.visMatrix_est(newPtsIds, nextFrame);
    matchedFeatInfo{1}   = Features_est{regId}(FeatIDinFrameReg);
    matchedFeatInfo{2}   = Features_est{nextFrame}(FeatIDinFrameNext);  
    matchedPDF{1}        = PDFCell{regId}(:,FeatIDinFrameReg);
    matchedPDF{2}        = PDFCell{nextFrame}(:,FeatIDinFrameNext);
    
    % Calculate Reconstruted Points.
    RelativeSturetureBag.MatchedFeature = matchedFeatInfo;
    RelativeSturetureBag.Matched3DPt_ID = newPtsIds;
    RelativeSturetureBag.poses          = StructureBag.poses;
    RelativeSturetureBag.f1             = regId;
    RelativeSturetureBag.f2             = nextFrame;        
    NewPts3D                            = ReconstrucPtsFromRelativePairFrames(RelativeSturetureBag,InfoBag);
    
    % Validate the reconstructed 3D points by PDD RPE.
    RelativeSturetureBag.Pts3D          = NewPts3D;   
    RelativeSturetureBag.matchedPDF     = matchedPDF;
    %[ValidID,~]     = SelectReconstructed3DPointbyRPE(RelativeSturetureBag,InfoBag, AlgoConfig);
    ValidID         = SelectReconstructed3DPtsByPDDRPE(RelativeSturetureBag,InfoBag, AlgoConfig);
    ValidNew3DPts   = NewPts3D(ValidID,:);
    ValidNewPtsID   = newPtsIds(ValidID);
        
    % Check if the reconstructed point is within the valid range.
    %ValidRangePtsID = ValidNew3DPts(:,3);
    
    
    
    % Check whether there is valid points left.
    if isempty(ValidNewPtsID)
       return;
    end
    
    % Update visMatrix and visBool according to the newly reconstructed 3D points.
    % check the matching relation between new 3D Point and the other registered frames.
    for n= 1:numel(ValidNewPtsID)
        Cur_3DPtsID    = ValidNewPtsID(n); % Obtain the current 3D point id.
        Cur_3DPtsCoord = ValidNew3DPts(n,:);
        qFramesID      = find(StructureBag.visBool_est(Cur_3DPtsID, :));                                 % Extract the ids of frames which can see the current point.
        qFramesID      = qFramesID( ismember( qFramesID, QueryImgInfo.registered ) );   % Extract the ids of registered frames which can see the current point.
        qFramesID      = qFramesID( ~ismember( qFramesID, [regId nextFrame] ) );        % Extract the ids of registered frames which can see the current point(not regid and nextframe).
        if ~isempty(qFramesID)
            for k = qFramesID                
                FrameInfoBag.CurPts        = Cur_3DPtsCoord;                
                FrameInfoBag.PDD           = PDFCell{k}( : , StructureBag.visMatrix_est( Cur_3DPtsID, k ) );
                FrameInfoBag.CurPoseMatrix = StructureBag.poses{k};
                OutlierFlag                = TestTrajectoryPDDRPE(FrameInfoBag,InfoBag,AlgoConfig);                
                if OutlierFlag==1   
                    StructureBag.visMatrix_est(Cur_3DPtsID, k) = 0;
                    StructureBag.visBool_est(Cur_3DPtsID, k) = 0;
                end                             
            end         
        end
    end
    
    % Update the Structure information.
    StructureBag.Pts3D          = [ StructureBag.Pts3D; ValidNew3DPts ];
    StructureBag.ValidPtsID     = [ StructureBag.ValidPtsID; ValidNewPtsID ];   
    
end
