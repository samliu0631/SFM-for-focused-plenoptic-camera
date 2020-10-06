function [visMatrix, visBool, lfFeatures] = filterRegisteredFrameFeatures(visMatrix, visBool, pts, pts3dIds, poses, frameId, lfFeatures, K, subLfExt)
% filter features of already reconstructed points based on reprojection
% error

outlReprTh = 1.5;

ptIds = find(visMatrix(pts3dIds, frameId));  % find the id of points seen by current frame within the reconstructed points.
pts = pts(:, ptIds); % Extract the  location of reconstructed points.
pts3dIds = pts3dIds(ptIds);  % The point id within  visMatrix.

for n = 1:numel(pts3dIds) % 遍历所有的重建点。
    
   feats = lfFeatures{frameId}{visMatrix(pts3dIds(n), frameId)}; % 获得当前帧的特征坐标
   errs = pointLfReprojectionError(pts(:, n), feats, poses{frameId}, subLfExt, K) ; % 计算重投影误差。
   
   rmIds = find(errs > outlReprTh);
   
   if isempty(rmIds)
       continue;
   end
   
   if (numel(rmIds) == size(feats, 1)) % 如果全部都是外点，则剔除当前特征在vismatrix中的记录。
       visMatrix(pts3dIds(n), frameId) = 0;
       visBool(pts3dIds(n), frameId) = 0;
   else
       
       feats(rmIds, 1) = inf(numel(rmIds), 1);
       feats = feats(~(feats(:, 1)==inf), :);
       lfFeatures{frameId}{visMatrix(pts3dIds(n), frameId)} = feats;  % 只保留帧内的RPE较低的特征点。
   end
    
end

