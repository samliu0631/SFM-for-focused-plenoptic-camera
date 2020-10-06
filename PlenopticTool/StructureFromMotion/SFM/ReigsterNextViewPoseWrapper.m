function  [RegisterFlag, StructureBag] = ReigsterNextViewPoseWrapper(StructureBag,QueryImgInfo,GeneratedDataBag,InfoBag, AlgoConfig)

    NextFrameInfo.nextframeID    = QueryImgInfo.nextFrame;
    NextFrameInfo.FeatureCoords  = GeneratedDataBag.Features_est{QueryImgInfo.nextFrame};
    NextFrameInfo.PDF            = GeneratedDataBag.PDFCell{QueryImgInfo.nextFrame};
    [RegisterFlag, StructureBag] = RegisterNextViewPose(NextFrameInfo, StructureBag, InfoBag, AlgoConfig);

end