function Pts3D = ReconstrucPtsFromRelativePairFrames(InitialStructureBag,InfoBag)
    f1             = InitialStructureBag.f1;
    f2             = InitialStructureBag.f2;  
    poses          = InitialStructureBag.poses;
    R1             = poses{f1}(1:3,1:3);                 % Ref frame is the same with the world coordinates.
    t1             = poses{f1}(1:3,4);
    R2             = poses{f2}(1:3,1:3);  %  Xc = RXw + t, from world coordinates to cameraa
    t2             = poses{f2}(1:3,4);
    MatchedFeature = InitialStructureBag.MatchedFeature;
    numPoints      = size(InitialStructureBag.MatchedFeature{1},1);
    Pts3D          = zeros(numPoints,3);     % Store the calculated 3D points.
    % convert the point in world coordinates to LF coordinates
    extMat1        = [R1' -R1'*t1; [zeros(1, 3) 1] ];
    extMat2        = [R2' -R2'*t2; [zeros(1, 3) 1] ];

    for pt = 1:numPoints  % go through all matched features
        % Get the number of ray for current matched feature in the ref frame.
        numMat1 = size( MatchedFeature{1}{pt},1);

        % Get the number of ray for current matched feature in the com frame.
        numMat2 = size( MatchedFeature{2}{pt},1);

        ProjMatrix = zeros(3, 4, numMat1 + numMat2); % the projection matrix for each virtual sub-camera.
        imPoints   = zeros(2, numMat1 + numMat2);    % the pixel coordinates in each sub-camera.

        CurrentFeature = MatchedFeature{1}{pt};

        for n = 1:numMat1  % 遍历前一帧具有特征的子孔径
            % get the id of micro-img.
            Current_LensID = CurrentFeature(n,3);
            FeatureCoords  = CurrentFeature(n,1:2);

            % calculate the projection matrix for the sub-camera
            Kv = CalculateVCamMatrix(InfoBag,Current_LensID);

            % calculate the position of the sub-camera.
            LocSubCamera = CalculateVCamLocation(InfoBag,Current_LensID);

            % calculate the pixel location of projected feature in sub-camera.
            FeatureRelativeLoc = CalculateFeatureLocInSubCamera(InfoBag,Current_LensID,FeatureCoords);

            % Store the projection matrix and feature location.
            ProjMatrix(:, :, n) = Kv  * [eye(3), -LocSubCamera] * extMat1;
            imPoints(:, n) = FeatureRelativeLoc';
        end

        CurrentFeature = MatchedFeature{2}{pt};

        for n =1:numMat2   % 遍历后一帧具有特征的子孔径
            % get the id of micro-img.
            Current_LensID = CurrentFeature(n,3);
            FeatureCoords  = CurrentFeature(n,1:2);

            % calculate the projection matrix for the sub-camera
            Kv = CalculateVCamMatrix(InfoBag,Current_LensID);

            % calculate the position of the sub-camera.
            LocSubCamera = CalculateVCamLocation(InfoBag,Current_LensID);

            % calculate the pixel location of projected feature in sub-camera.
            FeatureRelativeLoc = CalculateFeatureLocInSubCamera(InfoBag,Current_LensID,FeatureCoords);

            % Store the projection matrix and feature location.
            ProjMatrix(:, :, numMat1 + n) = Kv  * [eye(3), -LocSubCamera] * extMat2;
            imPoints(:, numMat1 + n) = FeatureRelativeLoc';

        end

        % Calculate the 3d point in the world system.
        %[pt3d, inliers] = nviewTriangulatePts(ProjMatrix, imPoints,[32;32],'rob-2','lm'); % 
 
        LensletGridModel = InfoBag.LensletGridModel;
        Rmi  = LensletGridModel.HSpacing/2;
        [pt3d, inliers] = nviewTriangulatePts(ProjMatrix, imPoints,[2*Rmi;2*Rmi],'dlt','lm');
        Pts3D(pt,:) = pt3d;
    end

end