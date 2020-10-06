function QueryImgInfo = InitialQueryImgInfo(GeneratedDataBag,InfoBag)

    LFnum                       = size(GeneratedDataBag.Features_est,1);
    QueryImgInfo.registered     = [GeneratedDataBag.f1,GeneratedDataBag.f2];
    QueryImgInfo.framesID       = 1:LFnum;
    QueryImgInfo.PDFCell        = GeneratedDataBag.PDFCell;                        % the PDF of all frames.
    QueryImgInfo.Features_est   = GeneratedDataBag.Features_est;  % the point features of all frames.
    QueryImgInfo.width          = InfoBag.pixelX;
    QueryImgInfo.height         = InfoBag.pixelY;
    QueryImgInfo.queryImgID     = QueryImgInfo.framesID(~ismember(QueryImgInfo.framesID, QueryImgInfo.registered));
    QueryImgInfo.notRegistered  = [];
    QueryImgInfo.badFrames      = [];
    QueryImgInfo.nextFrame      = [];
    QueryImgInfo.numRegistered  = 0;
    QueryImgInfo.looped         = 0;        % flag of loop.
    QueryImgInfo.f1             = GeneratedDataBag.f1;
    QueryImgInfo.f2             = GeneratedDataBag.f2;
end