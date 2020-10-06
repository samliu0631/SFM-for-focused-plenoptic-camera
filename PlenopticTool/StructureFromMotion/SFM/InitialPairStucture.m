function OutInitialStructureBag = InitialPairStucture(RelativeSturetureBag,InfoBag,AlgoConfig) 
 
 % Reconstructed 3D points.
 Pts3D                      = ReconstrucPtsFromRelativePairFrames(RelativeSturetureBag,InfoBag);
 
 %RelativeSturetureBag.Pts3D = Pts3D; 
 % validate the reconstructed 3D points.
 %[ValidPtsID,ReproErrors]   = SelectReconstructed3DPointbyRPE(RelativeSturetureBag,InfoBag, AlgoConfig); % ø…“‘…æ≥˝°£
 %Pts3D                      = Pts3D(ValidPtsID,:);
 %ValidPtsID                 = RelativeSturetureBag.Matched3DPt_ID(ValidPtsID); % the point ID within the initial visMatrix.

 % Output.
 OutInitialStructureBag.Pts3D         = Pts3D;
 OutInitialStructureBag.ValidPtsID    = RelativeSturetureBag.Matched3DPt_ID ;
 OutInitialStructureBag.visMatrix_est = RelativeSturetureBag.visMatrix_est;
 OutInitialStructureBag.visBool_est   = RelativeSturetureBag.visBool_est;
 OutInitialStructureBag.poses         = RelativeSturetureBag.poses;
end