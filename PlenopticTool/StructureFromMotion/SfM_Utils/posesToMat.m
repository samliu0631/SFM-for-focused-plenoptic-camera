function mat = posesToMat(poses)
%

nposes = numel(poses);
mat = zeros(nposes, 7);  % ����̬�任Ϊ 7Ϊ������  ��ת����任Ϊ  4Ԫ���� ƽ��������ȻΪƽ��������

for n = 1:nposes  %��������ע��֡
    R = poses{n}(1:3, 1:3);
    t = poses{n}(1:3, 4);
    t = t';
    q = rot2quat(R);
    mat(n, :) = [q' t];
    
end

end

