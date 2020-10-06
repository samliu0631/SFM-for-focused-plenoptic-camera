function [LocalVector,R0vector] = posesToVector(poses)
    nposes = numel(poses);
    LocalVector = zeros(nposes, 6);  % ����̬�任Ϊ6ά��������  ��ת����任Ϊ  4Ԫ���� ƽ��������ȻΪƽ��������
    R0vector = zeros(nposes,4);
    for n = 1:nposes  %��������ע��֡
        R1 = poses{n}(1:3, 1:3);
        t1 = poses{n}(1:3, 4);
        R  = R1';
        t = -R1'*t1;
        q = rot2quat(R);
        LocalVector(n, 4:6) = t';  % modified by sam  % ˵��������Ҫ��ɴ���������ϵ����ǰ����ϵ�����ꡣ
        R0vector(n,:)= q';
    end
    LocalVector = LocalVector';
    LocalVector = LocalVector(:);
    
    R0vector = R0vector';
    R0vector = R0vector(:);
end

