function [RegisterFlag, StructureBag] = RegisterNextViewPose(NextFrameInfo, StructureBag, InfoBag, AlgoConfig)
 
 % Calculate the absolute pose of the selected frames.    
 [NextFramePoseBag,StructureBag] = CalculateAbsPoseOfNextFrame(NextFrameInfo, StructureBag, InfoBag, AlgoConfig);
 RegisterFlag     = NextFramePoseBag.registerSucc;
 if RegisterFlag==0
     return;
 end
 
 % Remove outliers.
 %StructureBag = RemoveOutliersInAbsPose(NextFramePoseBag, StructureBag, InfoBag, AlgoConfig);
 
 % Remove outliers through PDD RPE, if inner number is too small,
 % set RegsiterFlag = false;
 [StructureBag,RegisterFlag] = RemoveOutliersByPDDRPE(NextFramePoseBag, StructureBag, InfoBag, AlgoConfig);

end

