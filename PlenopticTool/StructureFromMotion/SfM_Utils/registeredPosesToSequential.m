function [visMatrixOut, posesOut, lfFeaturesOut] = registeredPosesToSequential(visMatrix, poses, lfFeatures, registered, notRegistered)
% Map the registered poses to that of a sequential index of the cameras

n = numel(registered);  % 注册图像帧数。

lfFeaturesOut = cell(n ,1);  % 存储注册图像的特征位置信息。
posesOut = cell(n, 1);       % 记录

% we dont want to change the 1st dimension as we are indexing the
% reconstructed 3D points.
visMatrixOut = zeros(size(visMatrix, 1), n);  % 存储当前注册帧的visMatrix

for n = 1:numel(registered)  % 遍历注册的图像
    posesOut{n} = poses{registered(n)};    % 提取变换到世界坐标系的姿态
    lfFeaturesOut{n} = lfFeatures{registered(n)};% 提取注册帧的特征信息
    visMatrixOut(:, n) = visMatrix(:, registered(n));  %提取注册帧的visMatrix
end

end

