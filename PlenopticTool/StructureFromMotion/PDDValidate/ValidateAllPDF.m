function StructureBag = ValidateAllPDF(StructureBag, QueryImgInfo , InfoBag, AlgoConfig )
    flag               = 3; 
    maxPDDCenterError  = AlgoConfig.maxPDDCenterError_PtsRP;
    maxPDRError        = AlgoConfig.maxPDRError_PtsRP * InfoBag.LensletGridModel.HSpacing/2;
    % Extract data of registered frames.
    [visMatrixRegister, visBoolRegister , posesRegister, FeaturesRegister,PDFRegister] = ExtractRegisterFrameInfo(StructureBag, QueryImgInfo);

    % Data prepare for the SBA.
    Pts3D         = StructureBag.Pts3D; 
    npts          = size(Pts3D,1);           % number of the points.
    ncams         = size(visBoolRegister,2); % number of the cameras.
    if flag == 2
        mnp     = 2;  % number of parameter of the 3D projection plenoptic disc feature.
        cal         = [InfoBag.fx,  0,  InfoBag.cx,InfoBag.fy,InfoBag.cy];  % the camera parameters
    else
        mnp     = 3;  % if use PDF instead of 2D projection.
        Radius      = InfoBag.LensletGridModel.HSpacing/2;
        cal         = [InfoBag.fx,0,InfoBag.cx,InfoBag.fy,InfoBag.cy,InfoBag.K1,InfoBag.K2,Radius*2]; % the camera parameters of LF camera.
    end
    
    % prepare the initial paramter vector.
    [LocalVector,R0vector] = posesToVector(posesRegister);

    % Calculate the measurement vector x in SBA.
    camsInit     = reshape(LocalVector,6,ncams)';  
    PDFCenterErrorArray = -1*ones(npts,ncams);
    PDRErrorArray = -1*ones(npts,ncams);
    for id = 1:npts  % iterate all the 3D points.
        Curxyz      = Pts3D(id,:);                     % 当前3d点。
        FrameIDSeen = find(visBoolRegister(id, :)>0);  % Get the index of registered frames which see the current point.
        VisCamNum   = numel(FrameIDSeen);              % The number of frame seeing current 3D point.
        for i = 1:VisCamNum                            % iterate all cameras which see current point.
            CurFrameID  = FrameIDSeen(i);              % The current frame id.
            localrot    = camsInit(CurFrameID,1:6) ;   % 相机局部旋转的变量和方向。
            m           = projRTS( CurFrameID-1, 0, localrot, Curxyz, R0vector' , cal);          % Get the proj feature vector.
            FeatLoc     = PDFRegister{CurFrameID}(1:mnp,visMatrixRegister( id, CurFrameID ) );   % Get the detected feature vector .
            if flag ==3
                FeatLoc(3)  = FeatLoc(3)*Radius;    % multiply the the plenoptic disc radius with radius of micro-image.
            end
            PDFCenterError = sqrt(  sum( ( m(1:2) - FeatLoc( 1:2 )' ).^2 ) );
            PDRError       = sqrt(  sum( ( m(3)   - FeatLoc(3)     ).^2 ) );       
            
            PDFCenterErrorArray(id,CurFrameID) = PDFCenterError;
            PDRErrorArray(id,CurFrameID)       = PDRError;
            % if the projected PDD error is too large, remove the 3D point.
            if ( PDFCenterError> maxPDDCenterError) || (PDRError > maxPDRError )|| (Curxyz(3)> AlgoConfig.maxTriangleDist ) || (Curxyz(3)<0 )     %the error is different when after the SBA.                             
                %Set the 3d points with large PDD RPE as outliers .
                StructureBag.Pts3D(id,:)      = [-1,-1,-1];
                StructureBag.ValidPtsID(id,:) = -1;     
            end
        end        
    end
    
    % Calculate the triangle angle of each points.
    CameraCoord1 = StructureBag.poses{QueryImgInfo.f1}(:,4);
    CameraCoord2 = StructureBag.poses{QueryImgInfo.f2}(:,4);
    % Validate the reconstructed point by angle.
    Vector1  = CameraCoord1'- Pts3D;
    Vector2  = CameraCoord2'- Pts3D;
    
    NormVector1 = sqrt( sum(Vector1.^2, 2) );
    NormVector2 = sqrt( sum(Vector2.^2, 2) );
    
    costheta = sum( Vector1.*Vector2 , 2 )./(NormVector1.*NormVector2);
    theta = acos(costheta)/pi*180;
    InValidAngleID = ( (theta < AlgoConfig.maxAngle)==1);

    
    % Remove the 3d points with large PDD RPE.
    RemoveID =  sum(StructureBag.Pts3D==-1,2)==3;
    
    RemoveID = InValidAngleID|RemoveID ;
    StructureBag.Pts3D( RemoveID, :) =[];
    StructureBag.ValidPtsID(RemoveID) =[];
    
end