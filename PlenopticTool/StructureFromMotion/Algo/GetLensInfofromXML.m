function [LensletGridModel,GridCoords,ImgSize] = GetLensInfofromXML(WhitePath, XMLPath, FileSpecWhite)
    flag1 = exist([XMLPath,'/lensmodel.mat'],'file');
    if flag1==2
        load([XMLPath,'/lensmodel'], 'LensletGridModel', 'GridCoords','ImgSize');
    else
        ShowFlag                          = false;        
        InfoBag.FlagRemoveEdge            = false;
        
        % load the information about MLA.
        [FileList_raw, BasePath_raw]    = ReadRawImgInfo(WhitePath, FileSpecWhite);
        WhiteImage                      = ReadRawImg(BasePath_raw,FileList_raw, 1);        
        pixelX                          = size(WhiteImage,2);
        pixelY                          = size(WhiteImage,1);
        ImgSize                         = [pixelY,pixelX];
        
        [LensletGridModel, GridCoords] = FuncGenerateMLA([pixelY,pixelX], XMLPath ,ShowFlag,InfoBag);
        %[LensletGridModel,GridCoords,ImgSize]= GetMLAInfoByWhiteImg(WhitePath, FileSpecWhite,RoughRadius);
        save([XMLPath,'/lensmodel'], 'LensletGridModel', 'GridCoords','ImgSize');
    end
end