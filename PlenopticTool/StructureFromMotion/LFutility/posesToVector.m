function [LocalVector,R0vector] = posesToVector(poses)
    nposes = numel(poses);
    LocalVector = zeros(nposes, 6);  % 将姿态变换为6维度向量，  旋转矩阵变换为  4元数， 平移向量依然为平移向量。
    R0vector = zeros(nposes,4);
    for n = 1:nposes  %遍历所有注册帧
        R1 = poses{n}(1:3, 1:3);
        t1 = poses{n}(1:3, 4);
        R  = R1';
        t = -R1'*t1;
        q = rot2quat(R);
        LocalVector(n, 4:6) = t';  % modified by sam  % 说明这里需要变成从世界坐标系到当前坐标系的坐标。
        R0vector(n,:)= q';
    end
    LocalVector = LocalVector';
    LocalVector = LocalVector(:);
    
    R0vector = R0vector';
    R0vector = R0vector(:);
end

