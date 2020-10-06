function QueryImgInfo = UpdateQueryImgInfo_Fail2RegisterFrame(QueryImgInfo)
    QueryImgInfo.notRegistered = [QueryImgInfo.notRegistered, QueryImgInfo.nextFrame];
    QueryImgInfo.notRegistered = unique(QueryImgInfo.notRegistered);
    fprintf('Could not register frame %d \n', QueryImgInfo.nextFrame);
    tem_queryImgID = QueryImgInfo.framesID(~ismember(QueryImgInfo.framesID, QueryImgInfo.registered));
    tem_queryImgID = tem_queryImgID(~ismember(tem_queryImgID, QueryImgInfo.notRegistered));
    tem_queryImgID = tem_queryImgID(~ismember(tem_queryImgID, QueryImgInfo.badFrames));
    % update the queryImgID.
    QueryImgInfo.queryImgID = tem_queryImgID;
end
