function [allPts,allClearPts,allOmega ] = DetectCheckBoardCornerOnRawImg(CalibPath ,CalibFileSpec,GridCoords,LensletGridModel)
    ConfigBag.neighbrs                = 35;     % ROI range 
    ConfigBag.SearchThreshold         = 5;      % remove corenrs located on the edges of micro-images
    ConfigBag.Error_Inner             = 1.5;    % the 
    ConfigBag.ProjTresh               = 2;      % remove project points  on the edges of microimage.
    ConfigBag.delta                   = 0.5;    % Gaussian filter delta.
    ConfigBag.Optdelta                = 0.5;    % Gaussain filter delta used to filter raw image.
    ConfigBag.FilterRadius            = [2,4];  % the Raidus of corner detector.
    ConfigBag.OptThresh               = 5;      % The width of the edge in the micro images.
    ConfigBag.FilePath_Raw            = CalibPath;
    ConfigBag.DefaultFileSpec_Raw     = CalibFileSpec ;
    [allPts,allClearPts,allOmega ]    = DetectCornersOurs(GridCoords,LensletGridModel,ConfigBag);
    %ShowDetectResults(CalibPath, CalibFileSpec,allClearPts);
end