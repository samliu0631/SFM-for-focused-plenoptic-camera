function [paramFinal,FrameIdUsed] = GetCalibResults(CalibPath,CalibFileSpec,GridCoords,LensletGridModel,BoardCornerGap,BoardCornerXYNum)
    % Extract corner feature in the checkerboard raw images.
    flag1 = exist([CalibPath,'/OurNewDetectResults.mat'],'file');
    if flag1==2
        load([CalibPath,'/OurNewDetectResults.mat'], 'allPts','allClearPts','allOmega');
        %ShowDetectResults(CalibPath, CalibFileSpec,allClearPts);
    else
        [allPts,allClearPts,allOmega ] = DetectCheckBoardCornerOnRawImg(CalibPath ,CalibFileSpec,GridCoords,LensletGridModel);
        save([CalibPath,'/OurNewDetectResults.mat'], 'allPts','allClearPts','allOmega');
    end

    % Calibrate the plenoptic camera.
    [paramFinal,FrameIdUsed] = CalibPlenopticCameraUseCorner(allPts,allOmega,GridCoords,BoardCornerGap,BoardCornerXYNum);


end