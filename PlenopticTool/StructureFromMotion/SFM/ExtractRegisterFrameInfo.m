function [visMatrixRegister, visBoolRegister , posesRegister, FeaturesRegister,PDFRegister] = ExtractRegisterFrameInfo(StructureBag, QueryImgInfo)
    RegisterFrame    = QueryImgInfo.registered';    
    % 可能按顺序排列更好一点。
    posesRegister    = StructureBag.poses(RegisterFrame);
    FeaturesRegister = QueryImgInfo.Features_est(RegisterFrame);
    PDFRegister      = QueryImgInfo.PDFCell(RegisterFrame);
    ValidPtsID       = StructureBag.ValidPtsID;
    
    visMatrixRegister= StructureBag.visMatrix_est(ValidPtsID, RegisterFrame);
    visBoolRegister  = StructureBag.visBool_est(ValidPtsID, RegisterFrame);
end
