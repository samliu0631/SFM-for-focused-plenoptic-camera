function  [NextFramePoseBag,StructureBag] = CalculateAbsPoseOfNextFrame(NextFrameInfo, StructureBag, InfoBag, AlgoConfig)
    
    % Data Input.
    pts3dIds           = StructureBag.ValidPtsID;
    visMatrix          = StructureBag.visMatrix_est;
    visBool            = StructureBag.visBool_est;
    Pts3D              = StructureBag.Pts3D;
    nextFrame          = NextFrameInfo.nextframeID;    
    FeatureCoordsCell  = NextFrameInfo.FeatureCoords;
    RansacAbsTh        = AlgoConfig.RansacAbsTh;
    AbsPoseMaxIter     = AlgoConfig.AbsPoseMaxIter;
    
    % the ID of 3D point seen by current frame in pts3dIds.
    seenIds     = find(visBool(pts3dIds, nextFrame));   % the intial 3D points seen by current frames.
    FeatIDinFrame = visMatrix(pts3dIds(seenIds), nextFrame);
    MatchedFeatCoordsCell = FeatureCoordsCell(FeatIDinFrame);
    
    % Calculate the 6D ray coordinates for each ray.
    RayCell     = ConvertPoints2Ray({MatchedFeatCoordsCell},InfoBag);
    RayMat      = cell2mat(RayCell{1});
    
    % Extract the world coordinates of corresponding 3D points w.r.t. rays. 
    RayNum      = size(RayMat,1);
    ptsW        = zeros(3,RayNum); % Store the 3D point location .
    idsq        = zeros(RayNum,3); % Store the id of [3Dpoint, ray , FeatureInFrame].
    RayCounter  = 0;    
        
    for  n = 1:numel(seenIds)
        featId      = visMatrix(pts3dIds(seenIds(n)), nextFrame);
        PtsCurFrame = Pts3D(seenIds(n), :)';                 % 获得帧内当前特征对应匹配的 3D空间点。
        numRays     = size(FeatureCoordsCell{featId}, 1);   % number of rays for current feature.        
        ptsW(:,RayCounter+1:RayCounter+numRays)  = [repmat(PtsCurFrame, [1 numRays])];
        idsq(RayCounter+1:RayCounter+numRays,:)  = [ repmat(  pts3dIds( seenIds(n) ), [numRays 1] ) , (1:numRays)' ,  repmat( seenIds(n), [numRays 1] ) ]; 
        %  [ID of 3Dpoint in visBool, ID of ray ,ID of 3d point in reconstructed points ]   
        RayCounter  = RayCounter+numRays;      
    end
    
    % check if the current experiment is simulated 
    if isfield( InfoBag, 'SimulatedTestFlag' )
        if InfoBag.SimulatedTestFlag == true
            SimulatedTestFlag = true;
        else
            SimulatedTestFlag = false;
        end        
    else
        SimulatedTestFlag = false;
    end
    
    % check the input data.
    if  ~isempty(RayMat)     
        
        % carry out absolute pose estimation.
        if SimulatedTestFlag == true
            RtGT  = InfoBag.RtGT;
            [NextFramePoseBag,StructureBag] = SimulatedAbsPoseEst(ptsW, RayMat,RtGT,nextFrame);
            return;
        else
            [SuccessFlag , Xout] = RealAbsPoseEstimation(ptsW, RayMat,RansacAbsTh,AbsPoseMaxIter,nextFrame);
        end        
        
    else
        SuccessFlag = 0;        
    end       
    
    
    % Data Output.
    if SuccessFlag ==1
        NextFramePoseBag.Rw                 = Xout(1:3, 1:3);
        NextFramePoseBag.tw                 = Xout(1:3, 4);
        NextFramePoseBag.registerSucc       = 1;
        NextFramePoseBag.ptsW               = ptsW;
        NextFramePoseBag.RayMat             = RayMat;
        NextFramePoseBag.idsq               = idsq;
        %NextFramePoseBag.inlAbs             = inlAbs;
        NextFramePoseBag.Seen3dReconsPtsID  = seenIds;
        NextFramePoseBag.nextFrame          = NextFrameInfo.nextframeID;
        NextFramePoseBag.FeatureCoordsCell  = NextFrameInfo.FeatureCoords;
        NextFramePoseBag.PDF                = NextFrameInfo.PDF;
        NextFramePoseBag.FeatIDinFrame      = FeatIDinFrame;
        StructureBag.poses{nextFrame}       = [NextFramePoseBag.Rw, NextFramePoseBag.tw];
    else
        NextFramePoseBag.Rw                 = [];
        NextFramePoseBag.tw                 = [];
        NextFramePoseBag.registerSucc       = 0;
        NextFramePoseBag.ptsW               = ptsW;
        NextFramePoseBag.RayMat             = RayMat;
        NextFramePoseBag.idsq               = idsq;
        %NextFramePoseBag.inlAbs             = inlAbs;
        NextFramePoseBag.registerSucc       = 0;
        StructureBag.poses{nextFrame}       = [];
    end
    
    
end




function [NextFramePoseBag,StructureBag] = SimulatedAbsPoseEst(ptsW, RayMat,RtGT,nextFrame)
    % Calculate the absolute pose of current frame.
    MatchNum  = size(ptsW,2);
    
    % disturb the ground truth.
    t_gt = RtGT(1:3,4);
    R_gt = RtGT(1:3,1:3);
    [t_perturbed,R_perturbed] = perturb(t_gt, R_gt, 0.01);
    T_perturbed = [R_perturbed,t_perturbed];

    % calculate the absolute pose.
    [X, inlAbs] = opengvV2('gp3p_ransac',[1:1:MatchNum] ,ptsW, RayMat',T_perturbed);

    fprintf('frame %d: %.2f inliers\n', nextFrame, numel(inlAbs)/size(RayMat, 1));
    NextFramePoseBag = [];
    StructureBag.poses{nextFrame} = X;
end



function [SuccessFlag , Xout] = RealAbsPoseEstimation(ptsW, RayMat,RansacAbsTh,AbsPoseMaxIter,nextFrame)
        % Calculate the absolute pose of current frame.
        [X, inlAbs] = opengvV2('gp3p_ransac', ptsW, RayMat');
        fprintf('frame %d: %.2f inliers\n', nextFrame, numel(inlAbs)/size(RayMat, 1));
        
        % Iterate more time if the inlier is not enough.
        iter = 1;
        while (numel(inlAbs)/size(RayMat, 1) < RansacAbsTh) && iter< AbsPoseMaxIter
            [X, inlAbs] = opengvV2('gp3p_ransac', ptsW, RayMat');
            fprintf('frame %d: %.2f inliers\n', nextFrame, numel(inlAbs)/size(RayMat, 1));
            iter = iter + 1;
        end
        
        % If fail to calculate the absolute poses.
        if ( (iter==AbsPoseMaxIter) && ( numel(inlAbs)/size(RayMat, 1) < RansacAbsTh )  )
            SuccessFlag = 0;
            Xout = [];           
            return;
        end
        
        % Set the flag.
        SuccessFlag  = 1;
        
        % the results need to be further optimized by nonlinear algorithm。
        Xout         = opengv('abs_nonlin_noncentral', double(inlAbs), ptsW, RayMat', X);

end
