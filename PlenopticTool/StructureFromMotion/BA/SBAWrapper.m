function [QueryImgInfo,StructureBag] = SBAWrapper(StructureBag, QueryImgInfo,InfoBag,AlgoConfig,GeneratedDataBag)
    
    % SBA        
    if (numel(QueryImgInfo.registered) == 3 ||( numel(QueryImgInfo.registered) == 4))
        fprintf('Registered %d new images, calling global Bundle Adjustment...\n', numel(QueryImgInfo.registered));
        StructureBag = LFSBA(StructureBag, QueryImgInfo,InfoBag,GeneratedDataBag);
    end
    
    % SBA.
    if (QueryImgInfo.numRegistered == AlgoConfig.bundleBatch)
        fprintf('Registered %d new images, calling global Bundle Adjustment...\n', AlgoConfig.bundleBatch);
        StructureBag = LFSBA(StructureBag, QueryImgInfo,InfoBag,GeneratedDataBag);
        QueryImgInfo.numRegistered = 0;
    end

end