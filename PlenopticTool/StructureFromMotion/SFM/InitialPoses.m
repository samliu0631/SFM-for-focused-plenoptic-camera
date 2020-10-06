function OutInitialPoseBag = InitialPoses(InitPoseBag,InfoBag, varargin)
    if  nargin ==3
        AlgoConfig = varargin{1};
    end
    f1            = InitPoseBag.f1;
    f2            = InitPoseBag.f2;
    visBool_est   = InitPoseBag.visBool_est;
    visMatrix_est = InitPoseBag.visMatrix_est;
    Features_est  = InitPoseBag.Features_est;
    
    % Get the matching point for the image pair.
    ids = visBool_est(:, f1) .* visBool_est(:, f2);
    Matched3DPt_ID = find(ids);      % The ID of 3D point seen by both frames.

    % Get the feature id within each frames for each matched feature.
    FeatureIDRef   = visMatrix_est(Matched3DPt_ID,f1);
    FeatureIDCom   = visMatrix_est(Matched3DPt_ID,f2);

    % Get the location of matched features.
    MatchedFeature    = cell(2,1);
    MatchedFeature{1} = Features_est{f1}(FeatureIDRef,:);
    MatchedFeature{2} = Features_est{f2}(FeatureIDCom,:);

    % Convert the points to Ray.
    RayCellMatched = ConvertPoints2Ray(MatchedFeature,InfoBag);

    % calculate the relative poses using OpenGV.
    if isfield(InfoBag,'SimulatedTestFlag')
        if InfoBag.SimulatedTestFlag==true
           [R,t,InnerMatchID]  = CalculateRelativePose(RayCellMatched{1},RayCellMatched{2},InfoBag.RtGT);
        else
           [R,t,InnerMatchID]  = CalculateRelativePose(RayCellMatched{1},RayCellMatched{2});             
        end
    else
        [R,t,InnerMatchID]  = CalculateRelativePoseNew(RayCellMatched{1},RayCellMatched{2},AlgoConfig);
    end
        
    % Xref = RXcom+T
    
    % Remove Outliers.
    if ~isempty(R)
        
        % Validate the inner by RPE of PDD.        
        
        MatchedFeature{1} = MatchedFeature{1}(InnerMatchID,:);
        MatchedFeature{2} = MatchedFeature{2}(InnerMatchID,:);
        
        % updatae VisMatrix and visBool  
        %remove outliers in the visbool and visMatrix
        Ourlier3DPtsID = Matched3DPt_ID( ~ismember( [1:size(Matched3DPt_ID,1)], InnerMatchID) );        
        
        visMatrix_est(Ourlier3DPtsID,f1) = 0;
        visMatrix_est(Ourlier3DPtsID,f2) = 0;
        visBool_est(Ourlier3DPtsID,f1) = 0;
        visBool_est(Ourlier3DPtsID,f2) = 0;
        
        Matched3DPt_ID   = Matched3DPt_ID(InnerMatchID);
        
    end
   
    % Output data
    OutInitialPoseBag.R = R;
    OutInitialPoseBag.t = t;
    OutInitialPoseBag.MatchedFeature = MatchedFeature;
    OutInitialPoseBag.visMatrix_est      = visMatrix_est;
    OutInitialPoseBag.visBool_est        = visBool_est;
    OutInitialPoseBag.f1 = f1;
    OutInitialPoseBag.f2 = f2;
    OutInitialPoseBag.Matched3DPt_ID = Matched3DPt_ID;
end