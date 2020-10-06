function BAError = CalculateBAstructureError(InfoBag,StructureBag)
    GTPts     = InfoBag.Obj_WorldCoords';
    VisID     = InfoBag.VisID;
    GTPtsList = GTPts(VisID(StructureBag.ValidPtsID),:);
    BAError   = sum( sqrt ( sum( (GTPtsList - StructureBag.Pts3D).^2,2) ) );

end