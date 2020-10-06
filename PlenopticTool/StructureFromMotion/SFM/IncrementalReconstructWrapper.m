function [StructureBag,QueryImgInfo]= IncrementalReconstructWrapper(StructureBag,GeneratedDataBag, QueryImgInfo, InfoBag, AlgoConfig)
   
    for regId = QueryImgInfo.registered  % find the matching point of nextframe with all the registered frames.
        ImgIDPair = [regId QueryImgInfo.nextFrame];
        ImgIDPair = sort(ImgIDPair);
        if ~( GeneratedDataBag.geomVer_est( ImgIDPair(1), ImgIDPair(2) ) )
            fprintf('Skipping triangulation for unverified images..\n');
            continue;
        end
        % increamentally reconstruct
        QueryImgInfo.RegFrameID = regId;
        StructureBag            = incrementalReconstruct(StructureBag, QueryImgInfo, InfoBag, AlgoConfig);
    end
end