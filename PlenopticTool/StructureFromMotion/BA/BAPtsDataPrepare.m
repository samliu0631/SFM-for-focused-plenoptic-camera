function PtsCell  = BAPtsDataPrepare(StructureBag, InfoBag ,visMatrixRegister, FeaturesRegister)

    ValidPtsID   = StructureBag.ValidPtsID;
    Pts3D        = StructureBag.Pts3D;
    GridCoords   = InfoBag.GridCoords;
    NumPts       = size(ValidPtsID,1);   % Number of reconstructed points.
    PtsCell      = cell(NumPts,1);  
    SubCamSize   = numel(GridCoords(:,:,1));
    SubCamInfoSize = 2*SubCamSize+1;
    for id = 1:NumPts              % iterate through all reconstructed points.
        Cur_Pts  = Pts3D(id,:);    % the world coordinates of current point.
        CurPtsID = ValidPtsID(id); % the id of current points.
        FrameIDSeen = find(visMatrixRegister(CurPtsID, :)); % Get the index of registered frames which see the current point. 
        NumLfs    = numel(FrameIDSeen);
        Projs     = zeros(1, NumLfs*SubCamInfoSize);  % Store the projection info.
        for i = 1:NumLfs
           CurFrameID                 = FrameIDSeen(i);
           FeatLocInAllSubCam         = -ones(SubCamSize,2);
           FeatLocArray               =  FeaturesRegister{CurFrameID}{ visMatrixRegister( CurPtsID, CurFrameID ) };
           FeatLoc                    =  FeatLocArray(:,1:2);
           FeatLensID                 =  FeatLocArray(:,3);
           FeatLocInAllSubCam(FeatLensID,:) = FeatLoc;
           FeatLocInAllSubCam         = FeatLocInAllSubCam';
           FeatLocInAllSubCam         = FeatLocInAllSubCam(:)';
           Projs((i-1)*SubCamInfoSize+1:i*SubCamInfoSize) = [(CurFrameID-1), FeatLocInAllSubCam];
        end    
        PtsCell{id} = [Cur_Pts,NumLfs,Projs ];
    end 
end
