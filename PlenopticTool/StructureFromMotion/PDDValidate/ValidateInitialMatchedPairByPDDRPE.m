function OutInitialPoseBag = ValidateInitialMatchedPairByPDDRPE(OutInitialPoseBag, GeneratedDataBag,InfoBag,AlgoConfig)
% Set parameters
maxPDDCError  = AlgoConfig.maxPDDCenterError;
maxPDRError   = AlgoConfig.maxPDRError;

% Input parse.
R               = OutInitialPoseBag.R;
t               = OutInitialPoseBag.t;
PDFCell         = GeneratedDataBag.PDFCell;
f1              = OutInitialPoseBag.f1;
f2              = OutInitialPoseBag.f2;
Matched3DPt_ID  = OutInitialPoseBag.Matched3DPt_ID;
visMatrix_est   = OutInitialPoseBag.visMatrix_est;
K1              = InfoBag.K1;
K2              = InfoBag.K2;
fx              = InfoBag.fx;
fy              = InfoBag.fy;
cu              = InfoBag.cx;
cv              = InfoBag.cy;
% For all point in first frame.
PDD1stFrame  = PDFCell{f1}(:, visMatrix_est(Matched3DPt_ID,f1) );
PDD2stFrame  = PDFCell{f2}(:, visMatrix_est(Matched3DPt_ID,f2) );
PDDNum       = size(PDD1stFrame,2);


PDDCenterErrorList  = -1*ones(PDDNum,1);
PDRErrorList  = -1*ones(PDDNum,1);
OutMatchID   = [];
for i = 1 : PDDNum
    Cur1stPDD = PDD1stFrame(:,i);   
    Cur2stPDD = PDD2stFrame(:,i);  
    
    % constrain the PDR range within the valid range.
    if   ( abs(Cur1stPDD(3))< AlgoConfig.minValidPDR )|| (abs(Cur1stPDD(3))> AlgoConfig.maxValidPDR) || (abs(Cur2stPDD(3))< AlgoConfig.minValidPDR)|| (abs(Cur2stPDD(3))> AlgoConfig.maxValidPDR) 
        OutMatchID = [OutMatchID;i];  
        continue;
    end
    
    % Convert the PDD to 3D points in world coordinate.
    Pz = -K2/(Cur1stPDD(3)+K1);
    Px = Pz*(Cur1stPDD(1)-cu)/fx;
    Py = Pz*(Cur1stPDD(2)-cv)/fy;
    
    % Convert the 3D point to the second frame.
    P2st  = R'*[Px;Py;Pz]+(-R'*t);
    
    % Convert the 3D point to the PDD in second frame.    
    Cur1stPDDin2st = [fx*P2st(1)/P2st(3)+cu; fy*P2st(2)/P2st(3)+cv; -K2/P2st(3)-K1];
    
    % Calculate the PDD RPE.
    PDDCenterError = sqrt (  sum (  ( Cur2stPDD(1:2) - Cur1stPDDin2st(1:2) ).^2   ,1  )  );
    PDDRError      = abs(  Cur2stPDD(3) - Cur1stPDDin2st(3)  );
    
    % Store the error.
    PDDCenterErrorList(i) = PDDCenterError;
    PDRErrorList(i)       = PDDRError;
    
    % Check the error
    if (PDDCenterError > maxPDDCError) || (PDDRError > maxPDRError)
        OutMatchID = [OutMatchID;i];    
    end
end

% Update the  OutInitialPoseBag.Matched3DPt_ID
Out3DPts_id    = Matched3DPt_ID(OutMatchID);
Matched3DPt_ID(OutMatchID) = [];
OutInitialPoseBag.Matched3DPt_ID  = Matched3DPt_ID;

% Update the visBool and visMatrix
OutInitialPoseBag.visMatrix_est(Out3DPts_id ,f1) = 0;
OutInitialPoseBag.visMatrix_est(Out3DPts_id ,f2) = 0;
OutInitialPoseBag.visBool_est(Out3DPts_id ,f1)   = 0;
OutInitialPoseBag.visBool_est(Out3DPts_id ,f2)   = 0;

% Update the OutInitialPoseBag.MatchedFeature
ValidMatchedindex = ~ismember([1:PDDNum],OutMatchID);
OutInitialPoseBag.MatchedFeature{1} = OutInitialPoseBag.MatchedFeature{1}(ValidMatchedindex);
OutInitialPoseBag.MatchedFeature{2} = OutInitialPoseBag.MatchedFeature{2}(ValidMatchedindex);   
end