function cl = pointProjectionsToCell(pts, pts3dIds, visMatrix, lfFeatures)

npts = numel(pts3dIds); % 重建特征点的数量。
cl = cell(npts, 1);    % 存储所有重建特征点的 

LFsz = 1+25*2; % frame no + projections for one LF

for id = 1:npts  % 遍历所有重建特征点。
    ptId = pts3dIds(id);  % 重建特征点在visMatrix中的ID.
    
    pt = pts(:, id)';  % 获取当前重建3D点的世界坐标。
    
    lfSeen = find(visMatrix(ptId, :)); % 获取看到当前空间点的图像帧号。
    
    numLfs = numel(lfSeen);  % 看到当前空间点的 图像帧 数量。
    projs=zeros(1, numLfs*LFsz);
    for i = 1:numLfs   % 遍历所有 看到当前点的图像帧
        n = lfSeen(i);  % 当前图像帧号
        fs = -ones(25, 2);
        rs = lfFeatures{n}{visMatrix(ptId, n)}(:, 1:2);  % 获取当前点在当前图像帧的 匹配特征坐标。
%         fprintf('Point %d has %d projections in lf %d\n', id, size(rs, 1), n);
        subIds = lfFeatures{n}{visMatrix(ptId, n)}(:, 3:4);  % 获得当前点在当前图像帧的 匹配特征的子孔径编号。
        linIds = (subIds(:, 1) - 1 ) * 5 + subIds(:, 2);  % 获得子孔径图像的1维编号。
        fs(linIds, :) = rs;  % 将检测特征按照子孔径编号从1-25进行排序。
        fs = fs';
        fs = fs(:)';  % 将所有特征坐标压缩到1维向量中
        projs((i-1)*LFsz+1:i*LFsz)=[n-1 fs];
    end
    cl{id}=[pt numLfs projs];  % 记录 [ 当前点坐标 ， 看到的帧数，  在看到帧中的所有特征坐标构成的向量。 ]

end

end

