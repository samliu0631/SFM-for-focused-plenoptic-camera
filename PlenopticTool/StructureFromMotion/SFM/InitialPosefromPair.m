function OutInitialPoseBag = InitialPosefromPair(GeneratedDataBag,InfoBag, varargin)
    if  nargin ==3
        AlgoConfig = varargin{1};
    end

    fprintf('Start pose initialization...\n');
    InitPoseBag.f1            = GeneratedDataBag.f1;
    InitPoseBag.f2            = GeneratedDataBag.f2;
    InitPoseBag.visBool_est   = GeneratedDataBag.visBool_est;
    InitPoseBag.visMatrix_est = GeneratedDataBag.visMatrix_est;
    InitPoseBag.Features_est  = GeneratedDataBag.Features_est;
    
    if nargin==3
        OutInitialPoseBag     = InitialPoses(InitPoseBag,InfoBag,AlgoConfig);
    end    
    if  nargin==2
        OutInitialPoseBag     = InitialPoses(InitPoseBag,InfoBag);
    end
              
end