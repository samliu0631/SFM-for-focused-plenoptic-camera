function [visMatrixOut, posesOut, lfFeaturesOut] = registeredPosesToSequential(visMatrix, poses, lfFeatures, registered, notRegistered)
% Map the registered poses to that of a sequential index of the cameras

n = numel(registered);  % ע��ͼ��֡����

lfFeaturesOut = cell(n ,1);  % �洢ע��ͼ�������λ����Ϣ��
posesOut = cell(n, 1);       % ��¼

% we dont want to change the 1st dimension as we are indexing the
% reconstructed 3D points.
visMatrixOut = zeros(size(visMatrix, 1), n);  % �洢��ǰע��֡��visMatrix

for n = 1:numel(registered)  % ����ע���ͼ��
    posesOut{n} = poses{registered(n)};    % ��ȡ�任����������ϵ����̬
    lfFeaturesOut{n} = lfFeatures{registered(n)};% ��ȡע��֡��������Ϣ
    visMatrixOut(:, n) = visMatrix(:, registered(n));  %��ȡע��֡��visMatrix
end

end

