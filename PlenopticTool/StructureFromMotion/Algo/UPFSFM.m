function [StructureBag, QueryImgInfo, InfoBag, AlgoConfig] = UPFSFM(GeneratedDataBag,InfoBag)

    % algorithm configuration.
    AlgoConfig    = AlgoParamInit();

    % Initial Poses. 
    OutInitialPoseBag = InitialPosefromPair(GeneratedDataBag,InfoBag,AlgoConfig);  

    % Validate the matched feature by compare projection pdd and detected pdd.
    OutInitialPoseBag = ValidateInitialMatchedPairByPDDRPE(OutInitialPoseBag, GeneratedDataBag,InfoBag,AlgoConfig);    

    % Initial Structure. point cloud initialization.
    StructureBag      = InitialStructureFromPair(OutInitialPoseBag, GeneratedDataBag,InfoBag,AlgoConfig );

    % Prepare the info of the image query.
    QueryImgInfo      = InitialQueryImgInfo(GeneratedDataBag,InfoBag);

    %Validate the 3DPts and feature by compare projection pdd and detected pdd.
    StructureBag     = ValidateAllPDF(StructureBag, QueryImgInfo , InfoBag,AlgoConfig );

    % SBA after initialization.
    StructureBag     = LFSBA(StructureBag, QueryImgInfo,InfoBag);

    % start the loop.
    while(~isempty(QueryImgInfo.queryImgID))    
        % select the best next frame.
        QueryImgInfo = SelectNextViewFromQuerayImgInfo(StructureBag, QueryImgInfo, AlgoConfig);

        % if there is no more frame again, end the loop.
        if isnan(QueryImgInfo.nextFrame) && QueryImgInfo.looped   
            fprintf('No more images to register...\n');
            break;
        end

        % Register the pose of next frame and remove false matched 3D-2D.
        [RegisterFlag, StructureBag] = ReigsterNextViewPoseWrapper(StructureBag,QueryImgInfo,GeneratedDataBag,InfoBag, AlgoConfig);

        % Handle failed pose registration.
        if ~RegisterFlag
            QueryImgInfo = UpdateQueryImgInfo_Fail2RegisterFrame(QueryImgInfo);        
            continue;
        end

        % Increamentally Reconstruct 3D Points.
        [StructureBag,QueryImgInfo]= IncrementalReconstructWrapper(StructureBag,GeneratedDataBag, QueryImgInfo, InfoBag, AlgoConfig);

        % Register the frame ID.
        QueryImgInfo = RegisterCurrentFrame(QueryImgInfo);        

        % Invoke SBA.  
        [QueryImgInfo,StructureBag] = SBAWrapper(StructureBag, QueryImgInfo,InfoBag,AlgoConfig,GeneratedDataBag);    

    end

    % Visual Validation.
    if isfield(InfoBag,'SimulationFlag')
        if(InfoBag.SimulationFlag)
            VisualConfirm(InfoBag,StructureBag,true); % true when simulation
        else
            VisualConfirm(InfoBag,StructureBag); 
        end
    else
        VisualConfirm(InfoBag,StructureBag);
    end


end