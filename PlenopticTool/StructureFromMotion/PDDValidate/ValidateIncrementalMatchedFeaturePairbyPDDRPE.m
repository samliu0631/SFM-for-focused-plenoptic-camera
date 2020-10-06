function [StructureBag , ValidMatchedindex] = ValidateIncrementalMatchedFeaturePairbyPDDRPE(StructureBag,regId,nextFrame,InfoBag,PDFCell,newPtsIds,AlgoConfig)
    maxPDDCError  = AlgoConfig.maxPDDCenterError;
    maxPDRError   = AlgoConfig.maxPDRError;
    
    R1              = StructureBag.poses{regId}(1:3,1:3); % convert current points to world coordinates
    t1              = StructureBag.poses{regId}(1:3,4);
    R2              = StructureBag.poses{nextFrame}(1:3,1:3);
    t2              = StructureBag.poses{nextFrame}(1:3,4);
    K1              = InfoBag.K1;
    K2              = InfoBag.K2;
    fx              = InfoBag.fx;
    fy              = InfoBag.fy;
    cu              = InfoBag.cx;
    cv              = InfoBag.cy;
    
    % For all point in first frame.
    visMatrix          = StructureBag.visMatrix_est;
    FeatIDinFrameReg   = visMatrix(newPtsIds, regId);
    FeatIDinFrameNext  = visMatrix(newPtsIds, nextFrame);
    PDD1stFrame        = PDFCell{regId}(:, FeatIDinFrameReg );
    PDD2stFrame        = PDFCell{nextFrame}(:, FeatIDinFrameNext );
    PDDNum             = size(PDD1stFrame,2);

    OutMatchID   = [];
    for i = 1 : PDDNum
        Cur1stPDD = PDD1stFrame(:,i);
        Cur2stPDD = PDD2stFrame(:,i);

        if   ( abs(Cur1stPDD(3))< AlgoConfig.minValidPDR )|| (abs(Cur1stPDD(3))> AlgoConfig.maxValidPDR) || (abs(Cur2stPDD(3))< AlgoConfig.minValidPDR)|| (abs(Cur2stPDD(3))> AlgoConfig.maxValidPDR)
            OutMatchID = [OutMatchID;i];
            continue;
        end
        
        % Convert the PDD to 3D points in current camera coordinates.
        Pz = -K2/(Cur1stPDD(3)+K1);
        Px = Pz*(Cur1stPDD(1)-cu)/fx;
        Py = Pz*(Cur1stPDD(2)-cv)/fy;

        % Convert the 3D point to the world coordinates.
        Pworld  = R1*[Px;Py;Pz]+t1;

        % Convert the 3D point from world coordinates to the 2st camera
        % coordinates.
        P2st  = R2'*Pworld+(-R2'*t2);

        % Convert the 3D point to the PDD in second frame.
        Cur1stPDDin2st = [fx*P2st(1)/P2st(3)+cu; fy*P2st(2)/P2st(3)+cv; -K2/P2st(3)-K1];

        % Calculate the PDD RPE.
        PDDCenterError = sqrt (  sum (  ( Cur2stPDD(1:2) - Cur1stPDDin2st(1:2) ).^2   ,1  )  );
        PDDRError      = abs(  Cur2stPDD(3) - Cur1stPDDin2st(3)  );

        % Check the error
        if (PDDCenterError > maxPDDCError) || (PDDRError > maxPDRError)
            OutMatchID = [OutMatchID;i];
        end
    end

    Out3DPts_ID = newPtsIds(OutMatchID);

    % Update the visBool and visMatrix
    StructureBag.visMatrix_est(Out3DPts_ID,regId) = 0;
    StructureBag.visMatrix_est(Out3DPts_ID,nextFrame) = 0;
    StructureBag.visBool_est(Out3DPts_ID,regId) = 0;
    StructureBag.visBool_est(Out3DPts_ID,nextFrame) = 0;


    ValidMatchedindex = ~ismember([1:PDDNum],OutMatchID);

end