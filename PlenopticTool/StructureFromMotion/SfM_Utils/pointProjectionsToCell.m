function cl = pointProjectionsToCell(pts, pts3dIds, visMatrix, lfFeatures)

npts = numel(pts3dIds); % �ؽ��������������
cl = cell(npts, 1);    % �洢�����ؽ�������� 

LFsz = 1+25*2; % frame no + projections for one LF

for id = 1:npts  % ���������ؽ������㡣
    ptId = pts3dIds(id);  % �ؽ���������visMatrix�е�ID.
    
    pt = pts(:, id)';  % ��ȡ��ǰ�ؽ�3D����������ꡣ
    
    lfSeen = find(visMatrix(ptId, :)); % ��ȡ������ǰ�ռ���ͼ��֡�š�
    
    numLfs = numel(lfSeen);  % ������ǰ�ռ��� ͼ��֡ ������
    projs=zeros(1, numLfs*LFsz);
    for i = 1:numLfs   % �������� ������ǰ���ͼ��֡
        n = lfSeen(i);  % ��ǰͼ��֡��
        fs = -ones(25, 2);
        rs = lfFeatures{n}{visMatrix(ptId, n)}(:, 1:2);  % ��ȡ��ǰ���ڵ�ǰͼ��֡�� ƥ���������ꡣ
%         fprintf('Point %d has %d projections in lf %d\n', id, size(rs, 1), n);
        subIds = lfFeatures{n}{visMatrix(ptId, n)}(:, 3:4);  % ��õ�ǰ���ڵ�ǰͼ��֡�� ƥ���������ӿ׾���š�
        linIds = (subIds(:, 1) - 1 ) * 5 + subIds(:, 2);  % ����ӿ׾�ͼ���1ά��š�
        fs(linIds, :) = rs;  % ��������������ӿ׾���Ŵ�1-25��������
        fs = fs';
        fs = fs(:)';  % ��������������ѹ����1ά������
        projs((i-1)*LFsz+1:i*LFsz)=[n-1 fs];
    end
    cl{id}=[pt numLfs projs];  % ��¼ [ ��ǰ������ �� ������֡����  �ڿ���֡�е������������깹�ɵ������� ]

end

end

