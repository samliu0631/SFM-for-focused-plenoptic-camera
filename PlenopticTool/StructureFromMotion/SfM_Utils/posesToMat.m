function mat = posesToMat(poses)
%

nposes = numel(poses);
mat = zeros(nposes, 7);  % 将姿态变换为 7为向量，  旋转矩阵变换为  4元数， 平移向量依然为平移向量。

for n = 1:nposes  %遍历所有注册帧
    R = poses{n}(1:3, 1:3);
    t = poses{n}(1:3, 4);
    t = t';
    q = rot2quat(R);
    mat(n, :) = [q' t];
    
end

end

