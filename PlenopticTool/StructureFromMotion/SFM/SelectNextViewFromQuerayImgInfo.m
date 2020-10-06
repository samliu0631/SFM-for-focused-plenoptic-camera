function QueryImgInfo = SelectNextViewFromQuerayImgInfo(StructureBag, QueryImgInfo, AlgoConfig)
    [pyramids, nextFrame]  = SelectBestNextView(StructureBag, QueryImgInfo, AlgoConfig); 
    QueryImgInfo.nextFrame = nextFrame;
    
    if isnan(QueryImgInfo.nextFrame) && ~QueryImgInfo.looped  % if there is no more frame left, use the frames in the notRegistered again.
        QueryImgInfo.queryImgID    = QueryImgInfo.notRegistered;
        QueryImgInfo.notRegistered = [];
        QueryImgInfo.looped = 1;
    end
    
end