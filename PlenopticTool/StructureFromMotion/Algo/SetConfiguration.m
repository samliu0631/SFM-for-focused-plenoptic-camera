function InfoBag = SetConfiguration(FilePath_Raw)
    % set the parameters for raw image generation.
    
    % Set the working distance
    WorkDistance                = 1000;%1000; % 点云的大致位置。    
    FocusedVdValue              = 4;    % 以多大的虚深度观测。
    InfoBag.WorkDistance        = WorkDistance;
    
    % Set the scene points  
    InfoBag.Point3DNum          = 400; % useful when the scene is generated randomly.
    
    % Set the view points(LF camera) poses.
    InfoBag.Xc                  = [-10,  20,  0 , 0 ];%, 20,    10  ]; 
    InfoBag.Yc                  = [-20, 10, 17 ,0 ];%, 50 ,   0  ];
    InfoBag.Zc                  = [100,   0, 100, 0  ];%, 10 ,  10 ];
    InfoBag.ThetaXArray         = [0,  0,  0 , 0  ];%, 0,     0  ]; 
    InfoBag.ThetaYArray         = [0,  0,  0 , 0  ];%, 0,   -pi/80 ]; 
    InfoBag.ThetaZArray         = [0,  0,  0 , 0  ];%, pi/40,   0 ]; 
     
    % set the view point randomly.
%     LFframeNum   = 5;
%     XYRange      = 100;
%     RotateRange  = pi/10;
%     XYZCoords    = XYRange.*(rand(3,LFframeNum)-0.5).*2 ;   
%     RotateAngle  = RotateRange.*(rand(3,LFframeNum)-0.5).*2 ;   
%     InfoBag.Xc                  = [0, XYZCoords(1,:) ]; 
%     InfoBag.Yc                  = [0, XYZCoords(2,:) ] ; 
%     InfoBag.Zc                  = [0, XYZCoords(3,:) ]; 
%     InfoBag.ThetaXArray         = [0, RotateAngle(1,:)]; 
%     InfoBag.ThetaYArray         = [0, RotateAngle(2,:)]; 
%     InfoBag.ThetaZArray         = [0, RotateAngle(3,:)]; 
    
    
    % Set the intrinsic parameters  
    % TO DO  change the simulated parameters to focused plenoptic camera.
    InfoBag.pixelY              = 2000;
    InfoBag.pixelX              = 3000;
    InfoBag.sxy                 = 0.0055;              % size of  the pixel on sensor
    InfoBag.F                   = 35;                  % focal length(mm).
    InfoBag.B                   = 1.32;                   % Distance between MLA and sensor.
    %InfoBag.fm1                 = 2;                   % focal length of micro-lens type 1
    %InfoBag.fm2                 = 2;                   % focal length of micro-lens type 2
    %InfoBag.fm3                 = 2;                   % focal length of micro-lens type 3
    InfoBag.Dmi                 = 32;                  % the diameter of micro-image(pixel)
    InfoBag.k1                  = 0;
    InfoBag.k2                  = 0;
    InfoBag.bL0                 = InfoBag.F * WorkDistance / ( WorkDistance-InfoBag.F )-FocusedVdValue*InfoBag.B; % Distance between MLA and main lens.
    InfoBag.DL                  = 0.9*InfoBag.bL0/( InfoBag.B/(InfoBag.Dmi*InfoBag.sxy) );                        % the diameter of main lens aperture.
    
    % Calculate the rest of parameters
    InfoBag.Dml                 = InfoBag.Dmi*InfoBag.bL0/(InfoBag.bL0+InfoBag.B);  % The diameter of the micro-lenses.
    Lm                          = -InfoBag.bL0;
    Lc                          = -InfoBag.bL0-InfoBag.B;
    InfoBag.K1                  = -(Lm+InfoBag.F)*Lc/( (Lm-Lc)*InfoBag.F  );
    InfoBag.K2                  = Lm*Lc/(Lm-Lc);
    InfoBag.fx                  = (InfoBag.bL0+InfoBag.B)/InfoBag.sxy;
    InfoBag.fy                  = (InfoBag.bL0+InfoBag.B)/InfoBag.sxy;
    InfoBag.cx                  = InfoBag.pixelX/2;
    InfoBag.cy                  = InfoBag.pixelY/2;
   
    % Set the parameters for simulation.
    InfoBag.NumMainRay          = 51;               % the sample number of main lens
    InfoBag.DataPath            = FilePath_Raw;     % the data directory.
    InfoBag.FlagRemoveEdge      = false;            % whether remove the micro-lenses which is not complete during simulating Image.
    InfoBag.ParallelFlag        = false;            % whether use parallel computation during simulating image.
end
