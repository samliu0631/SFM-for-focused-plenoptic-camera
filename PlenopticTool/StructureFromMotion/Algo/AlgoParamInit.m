function  AlgoConfig = AlgoParamInit()
    AlgoConfig.pyramidRes               = 3;   % number of levels in the pyramid of next view selection    
    AlgoConfig.AbsPoseMaxIter           = 20;  % The number of iteration of absolute pose estimation.
    AlgoConfig.ransacRelTh              = 0.85;%0.9  % the inner ratio of the relative pose estimation.
    AlgoConfig.RansacAbsTh              = 0.2;  % the inner ratio of the absolute pose estimation.
    AlgoConfig.bundleBatch              = 5; % when the number of newly register frame reach this value, SBA will be called.
    AlgoConfig.Min3D2DMatchNum          = 20;    % The minimum 3D-2D match number used to estimate the frame's absolute pose.
    
    AlgoConfig.maxPDDCenterError        = 100;    %80;   % The reprojection error threshhold of PDD center used to define inner point.
    AlgoConfig.maxPDRError              = 1;      %1;    % The reprojection error threshhold of PDR used to define inner point.
    AlgoConfig.maxPDDCenterError_PtsRP  = 50;     %20;   % used to 
    AlgoConfig.maxPDRError_PtsRP        = 0.5;    %0.1;
    AlgoConfig.maxAngle                 = 1; % degree.
        
    AlgoConfig.maxTriangleDist          = 6000;
    AlgoConfig.maxIncreTriangleDist     = 6000;
    AlgoConfig.minValidPDR              = 2.8;
    AlgoConfig.maxValidPDR              = 30;
    
    
  end