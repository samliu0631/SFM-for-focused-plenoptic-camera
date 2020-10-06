function StructureBag = InitialStructureFromPair(OutInitialPoseBag, GeneratedDataBag,InfoBag,AlgoConfig )
    fprintf('Start structure initialization...\n');
    RelativeSturetureBag        = OutInitialPoseBag;
    LFnum                       = size(GeneratedDataBag.Features_est,1);
    poses                       = cell(LFnum, 1);         % Store the pose matrix from camera coordinate to world coordinate¡£
    poses{GeneratedDataBag.f1}  = [eye(3), zeros(3,1)]; 
    poses{GeneratedDataBag.f2}  = [OutInitialPoseBag.R ,OutInitialPoseBag.t];
    RelativeSturetureBag.poses  = poses;
    StructureBag                = InitialPairStucture(RelativeSturetureBag,InfoBag,AlgoConfig);
      
end