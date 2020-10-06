function QueryImgInfo = RegisterCurrentFrame(QueryImgInfo)
    QueryImgInfo.registered    = [QueryImgInfo.registered, QueryImgInfo.nextFrame];
    QueryImgInfo.numRegistered = QueryImgInfo.numRegistered+1;
    QueryImgInfo.notRegistered = QueryImgInfo.notRegistered(~ismember(QueryImgInfo.notRegistered, QueryImgInfo.registered));
    QueryImgInfo.queryImgID    = QueryImgInfo.framesID(~ismember(QueryImgInfo.framesID, QueryImgInfo.registered));
    % when successfully register a new frame, the queryImgID is updated.
    % It can include the notRegistered frame again.
    fprintf('Registered images: %d \n', QueryImgInfo.nextFrame);
end