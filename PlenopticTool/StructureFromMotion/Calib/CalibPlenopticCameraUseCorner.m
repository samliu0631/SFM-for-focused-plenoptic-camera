function [paramFinal,FrameIdUsed] = CalibPlenopticCameraUseCorner(allPts,allOmega,GridCoords,BoardCornerGap,BoardCornerXYNum)
    ShowFlag                 = true;
    NonLinearFlag            = true;
    ParallelFlag             = false;
    OmegaCell                = ConvertOmegaFormat(allOmega);
    [param,FrameIdUsed]      = ZhangCalibration(OmegaCell ,BoardCornerGap, BoardCornerXYNum ,ShowFlag); % zhang zhengyou Calibration
    [~,paramFinal]           = DepthCalibration( param,FrameIdUsed,allPts,OmegaCell,BoardCornerGap,BoardCornerXYNum,GridCoords,ShowFlag,NonLinearFlag,ParallelFlag);
    paramFinal  = paramFinal(1:end-3);
end