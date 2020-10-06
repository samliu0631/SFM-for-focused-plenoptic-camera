%Function:Select valid reconstruced 3D points by measurement the
%Reprojection error 
function [ValidPtsID,ReproErrors]= SelectReconstructed3DPointbyRPE(RelativeSturetureBag,InfoBag,AlgoConfig)
    % data prepare.
    GridCoords    = InfoBag.GridCoords;
    GridCoordsX   = GridCoords(:,:,1);
    GridCoordsY   = GridCoords(:,:,2);
    MatchedFeature= RelativeSturetureBag.MatchedFeature; % Get the estimated feature location.
    Pts3D         = RelativeSturetureBag.Pts3D;    
    poses         = RelativeSturetureBag.poses;
    f1            = RelativeSturetureBag.f1;
    f2            = RelativeSturetureBag.f2;
    Rh            = poses{f1}(1:3, 1:3);  % 获得当前参考帧到世界坐标系的位姿变换矩阵
    th            = poses{f1}(1:3, 4);
    extMat1       = [Rh' -Rh'*th; zeros(1, 3) 1];
    Rh            = poses{f2}(1:3, 1:3);   % 获得当前对照帧到世界坐标系的位姿变换矩阵。
    th            = poses{f2}(1:3, 4);
    extMat2       = [Rh' -Rh'*th; zeros(1, 3) 1];
    
    % calculate the camera coorinates of 3D pts in each frames.
    NumPts            = size(Pts3D,1);
    PtsCameraCoords1  = extMat1*[Pts3D';ones(1,NumPts)];
    PtsCameraCoords2  = extMat2*[Pts3D';ones(1,NumPts)];
    PtsCameraCoords   = cell(2,1);
    PtsCameraCoords{1}=  PtsCameraCoords1(1:3,:);
    PtsCameraCoords{2}=  PtsCameraCoords2(1:3,:);
    
    % calculate the reprojection error for each 3D points.
    ReproErrors  = zeros(NumPts,1);
    for i = 1:size(MatchedFeature,1)          % iterate two frames
        FeatureInfoCell = MatchedFeature{i};  % feature info for current frames.
        CurPtsCamCoords = PtsCameraCoords{i}; % Camera coordinates for all points.
        for j = 1:NumPts                      % go through all matched features.
            Cur_FeatureInfo = FeatureInfoCell{j};
            if isempty(Cur_FeatureInfo)
                ReproErrors(j)= ReproErrors(j)+AlgoConfig.InitStrutRPE*2;
                continue;
            end
            Cur_PtsLensID = Cur_FeatureInfo(:,3);
            ucs           = GridCoordsX(Cur_PtsLensID);                                             % 微透镜中心X像素坐标。
            vcs           = GridCoordsY(Cur_PtsLensID);
            du            = InfoBag.fx * CurPtsCamCoords(1, j) - CurPtsCamCoords(3, j).*(ucs - InfoBag.cx);
            dv            = InfoBag.fy * CurPtsCamCoords(2, j) - CurPtsCamCoords(3, j).*(vcs - InfoBag.cy);
            nominator     = 1./(InfoBag.K1 * CurPtsCamCoords(3, j) + InfoBag.K2);
            du            = nominator.*du;
            dv            = nominator.*dv;
            us            = ucs+du;
            vs            = vcs+dv;
            err           = [us,vs]-Cur_FeatureInfo(:,1:2);   % calculate reprojection error.
            err           = mean( sqrt( sum( err.^2 , 2 ) ) );
            ReproErrors(j)= ReproErrors(j)+err;
        end
    end
    ReproErrors = ReproErrors./size(MatchedFeature,1); % Reprejection error for each reconstructed 3D point.
    ValidPtsID = find(ReproErrors < AlgoConfig.InitStrutRPE); % TODO; Maybe this value should be less??
    % id of validated reconstructed 3d points.
end