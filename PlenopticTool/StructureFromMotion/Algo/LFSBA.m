function StructureBag = LFSBA(StructureBag, QueryImgInfo,InfoBag,GeneratedDataBag)
    flag = 3; % or 3 / 2  

    % Extract data of registered frames.
    [visMatrixRegister, visBoolRegister , posesRegister, FeaturesRegister,PDFRegister] = ExtractRegisterFrameInfo(StructureBag, QueryImgInfo);
          
    % Data prepare for the SBA.
    Pts3D       = StructureBag.Pts3D;
    npts        = size(Pts3D,1);           % number of the points.
    Pts3DT      = Pts3D';
    Pts3DVector = Pts3DT(:);               % vector of 3D coordinates of reconstructed points.
    ncams       = size(visBoolRegister,2); % number of the cameras.       
    cnp         = 6;  % number of parameters of the camera. fx, fy,cu,cv, galma.
    ncon        = 0;  % number of points remain constant.
    mcon        = 1;  % number of frame remian constant.
    pnp         = 3;  % number of parameter of the 3D point.
    if flag == 2        
        mnp     = 2;  % number of parameter of the 3D projection plenoptic disc feature.
    else
        mnp     = 3;  % if use PDF instead of 2D projection.
    end
    spmask      = sparse(visBoolRegister);  
    spmask      = double(spmask);  % The sparse visible matrix. The outliers need to be deleted in order to ensure the results of SBA.
    opts        = [1E-03, 1E-12, 1E-12, 1E-12, 0];% [t,e1,e2,e3,e4] the default configuration for SBA. 
    itmax       = 100;  % max iteration times
    verbose     = 1;   % nonuse for current codes, used as info output. 
    if flag == 2
        cal         = [InfoBag.fx,  0,  InfoBag.cx,InfoBag.fy,InfoBag.cy];  % the camera parameters
    else
        Radius      = InfoBag.LensletGridModel.HSpacing/2;
        cal         = [InfoBag.fx,0,InfoBag.cx,InfoBag.fy,InfoBag.cy,InfoBag.K1,InfoBag.K2,Radius*2]; % the camera parameters of LF camera.
    end
    
    % prepare the initial paramter vector.
    [LocalVector,R0vector] = posesToVector(posesRegister); 
    p0                     = [LocalVector',Pts3DVector'];  % the initial parameter vector.   
    
      
    % Calculate the measurement vector x in SBA.   
    Vis2DPtsNum  = sum( sum ( ( visBoolRegister>0 ) ) );     % Get the number of visible 2D points.
    ptsPDF       = zeros(1,mnp*Vis2DPtsNum);
    count        = 0;
    for id = 1:npts  % iterate all the 3D points.
        FrameIDSeen = find(visMatrixRegister(id, :)); % Get the index of registered frames which see the current point. 
        NumLfs      = numel(FrameIDSeen);             % The number of frame seeing current 3D point.
        Projs       = zeros(1, NumLfs*mnp);           % Store the projection info.
        for i = 1:NumLfs   % iterate all cameras which see current point.
           CurFrameID  = FrameIDSeen(i);              % The current frame id.
           FeatLoc     = PDFRegister{CurFrameID}(1:mnp,visMatrixRegister( id, CurFrameID ) );
           % 这里要判断特征数量，即是说只有光场圆域半径达标，才可以进行SBA.
           
           if flag ==3
             FeatLoc(3)  = FeatLoc(3)*Radius;    % multiply the the plenoptic disc radius with radius of micro-image. 
             %FeatLoc(3)  = FeatLoc(3)*Radius*50;
           end
           Projs( (i-1)*mnp+1 : i*mnp )= FeatLoc';    % Store the project coordinates.
        end
        ptsPDF( count+1 : count+ NumLfs*mnp) = Projs;
        count = count + NumLfs*mnp;
    end
        
         
    
    % Invoke SBA. [ret,p,info]= sba()  
    if flag == 2       
        [~, p, ~]= sba(npts, ncon, ncams, mcon, spmask, p0, cnp, pnp, ptsPDF, mnp, 'projRTS2D','jacprojRTS2D', itmax, verbose, opts, 'motstr', R0vector', cal);
    else
        [~, p, ~]= sba(npts, ncon, ncams, mcon, spmask, p0, cnp, pnp, ptsPDF, mnp, 'projRTS','jacprojRTS', itmax, verbose, opts, 'motstr', R0vector', cal);
        %[~, p, ~]= sba(npts, ncon, ncams, mcon, spmask, p0, cnp, pnp, ptsPDF, mnp, 'projRTS50', itmax, verbose, opts, 'motstr', R0vector', cal);
        %[~, p, ~]= sba(npts, ncon, ncams, mcon, spmask, p0, cnp, pnp, ptsPDF, mnp, 'projRTS', itmax, verbose, opts, 'str', R0vector', cal);
    end
    
    
%     % save paramters to be validated.
%     pts3DOrigin = Pts3D;      % Np*3
%     dmask = visBoolRegister;  % Np*Nc
%     r0    = R0vector';         % 1*Nc*4
%     camsInit = reshape(LocalVector,6,ncams)';   % Nc*6;
%     pts2D = ptsPDF;           % 1*VisPtsNum*2;
%     save('test.mat','dmask','cal','pts2D','camsInit','pts3DOrigin','r0','p');
%     
    
     % Update Results.
    StructureBag = UpdateResultFromSBA(p,ncams,cnp,npts,pnp,R0vector,StructureBag,QueryImgInfo);
    
end


