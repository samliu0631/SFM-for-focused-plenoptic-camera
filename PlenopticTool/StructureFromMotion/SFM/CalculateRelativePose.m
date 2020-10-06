function [R,t,InnerMatchID] = CalculateRelativePose(RayCellMatched_Ref,RayCellMatched_Com,varargin)
refineFlag         = true;  % whether use the nonlinear pose refine.
ransacRelTh        = 0.90;  %0.75;   % the inner ratio threshhold of relative pose estimation.
MachRatioMin       = 0.8;   % the ratio of inner point of a successfully matched PDF.
%innerNumberThresh  = 10000;  % the inner number of matched ray pair.

% prepare ray data for OpenGV.
[raysRef, raysCom, RaysIdsRef, ~ ] = PrepareRayCoordsForOpenGV(RayCellMatched_Ref , RayCellMatched_Com);

% check the input data
if isempty(raysRef)||isempty(raysCom)
    R=[];
    t=[];
    InnerMatchID=[];
    return;    
end

% Calculate the relative poses.  
if nargin == 3
    % used for simulated experiment.
    % Get the ground truth.
    RtGT = varargin{1};    
    % disturbed ground truth
    t_gt = RtGT(1:3,4);
    R_gt = RtGT(1:3,1:3);
    [t_perturbed,R_perturbed] = perturb(t_gt, R_gt, 0.01);
    T_perturbed = [R_perturbed,t_perturbed];    
    [TransformMatrix, inliers17pt] = opengv('seventeenpt_ransac', [1:1:size(raysRef,1)],raysRef', raysCom',T_perturbed);  % 使用OpenGV中的17点算法进行计算。
    fprintf('Inliers: %f...，Inlier number: %f \n', numel(inliers17pt) / size(raysRef, 1) , numel(inliers17pt));  % 输出内点比例
    R = TransformMatrix(1:3, 1:3);
    t = TransformMatrix(1:3, 4);
    InnerMatchID =[];
    return;        
%     if (numel(inliers17pt) / size(raysRef, 1) < 0.6) ||( numel(inliers17pt) < 1000 )
%         R =[];
%         t =[];
%         InnerMatchID =[];
%         return;        
%     else
%         R = TransformMatrix(1:3, 1:3);
%         t = TransformMatrix(1:3, 4);
%         InnerMatchID =[];
%         return;
%     end        
else
    [TransformMatrix, inliers17pt] = opengvV2('seventeenpt_ransac', raysRef', raysCom');  % 使用OpenGV中的17点算法进行计算。
    fprintf('Inliers: %f...，Inlier number: %f \n', numel(inliers17pt) / size(raysRef, 1) , numel(inliers17pt));  % 输出内点比例
    
    counter = 0;
    while ( numel(inliers17pt) / size(raysRef, 1 ) < ransacRelTh)
        [TransformMatrix, inliers17pt] = opengvV2('seventeenpt_ransac', raysRef', raysCom');
        fprintf('Inliers: %f...，Inlier number: %f \n', numel(inliers17pt) / size(raysRef, 1) , numel(inliers17pt));
        counter = counter+1;
        if counter == 50 % control the maximum iteration times
            R=[];
            t=[];
            InnerMatchID=[];
            return;
        end
    end    
    
end

% Refine the relative pose.
if  refineFlag == true  % the nonlinear refinement is really important.
    Xout = opengv('rel_nonlin_noncentral', double(inliers17pt), raysRef', raysCom', TransformMatrix);  % 使用非线性方法优化位姿估计结果。
    R = Xout(1:3, 1:3);
    t = Xout(1:3, 4);
else
    R = TransformMatrix(1:3, 1:3);
    t = TransformMatrix(1:3, 4);
end


% extract the id of inner matched points within matched frames.
InnerIds1 = RaysIdsRef(inliers17pt, :);   % select the inner ray id in reference frame.
InnerMatchID = unique(InnerIds1(:,1));   % inner matched features ID within matched frames.
MatchNum = size(InnerMatchID,1);
InnerMatchRatiao = zeros(MatchNum,1);    % store the successfully matching times for each feature pair.
for i=1:MatchNum
    CurrentInnerID = InnerMatchID(i);
    Num = sum( (InnerIds1(:,1)==CurrentInnerID) ); 
    NumOrigin = sum(RaysIdsRef(:,1)==CurrentInnerID );
    InnerMatchRatiao(i)= Num./NumOrigin;
end

InnerMatchID = InnerMatchID(InnerMatchRatiao> MachRatioMin ); 
end