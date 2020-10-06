function VisualConfirm(InfoBag,StructureBag,varargin)

    figure; hold on; 
    if nargin==3 && varargin{1}==true
        PosesGT = GetGtCamPoses(InfoBag);
    end
    

    CamNum  = size(StructureBag.poses,1);
    CamPose = StructureBag.poses;
    cameraSize = 12;
    for i=1:CamNum
        CameraID      = num2str(i);
        CamPoseMatrix = CamPose{i};        
        if  ~isempty(CamPoseMatrix)              
            if i == 1
                plotCamera('Location',CamPoseMatrix(1:3,4), 'Orientation',CamPoseMatrix(1:3,1:3)','Size', cameraSize,  'Opacity', 0,'Color',[0,1,0]); 
            else
                plotCamera('Location',CamPoseMatrix(1:3,4), 'Orientation',CamPoseMatrix(1:3,1:3)','Size', cameraSize,  'Opacity', 0,'Color',[1,0,0]); %,,'Label', CameraID
            end
            %plotCamera('Location',-CamPoseMatrix(1:3,1:3)'*CamPoseMatrix(1:3,4), 'Orientation',CamPoseMatrix(1:3,1:3)','Size', cameraSize,  'Opacity', 0,'Color',[1,0,0],'Label', CameraID); 
        end
        if nargin==3 && varargin{1}==true
            CamPoseGT     = PosesGT{i};
            %plotCamera('Location',CamPoseGT(1:3,4), 'Orientation',CamPoseGT(1:3,1:3),'Size', cameraSize, 'Label', CameraID, 'Opacity', 0,'Color',[0,1,0]);
            plotCamera('Location',-CamPoseGT(1:3,1:3)'*CamPoseGT(1:3,4), 'Orientation',CamPoseGT(1:3,1:3)','Size', cameraSize, 'Label', CameraID, 'Opacity', 0,'Color',[0,1,0]);
        end
        
    end

    % Check if there is pixel value.
    if isfield(StructureBag, 'Pts3DColor')
        MyPointcloud  = pointCloud(StructureBag.Pts3D,'Color',StructureBag.Pts3DColor);
    else
        MyPointcloud  = pointCloud(StructureBag.Pts3D);
    end
    
    pcshow(MyPointcloud,'MarkerSize',8);
    
    
    if nargin==3 && varargin{1}==true
        GTPts     = InfoBag.Obj_WorldCoords';
        VisID     = InfoBag.VisID;
        GTPtsList = GTPts(VisID(StructureBag.ValidPtsID),:);
        plot3(  GTPtsList(:,1),GTPtsList(:,2),GTPtsList(:,3), 'r*');
    end
    xlabel('X(mm)');
    ylabel('Y(mm)');
    zlabel('Z(mm)');
    hold off;
    axis tight
    grid on;
    %axis off;
    


end