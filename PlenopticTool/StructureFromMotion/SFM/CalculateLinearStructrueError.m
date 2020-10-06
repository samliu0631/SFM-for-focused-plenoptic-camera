function InitError = CalculateLinearStructrueError(StructureBag,InfoBag)
    GTPts     = InfoBag.Obj_WorldCoords';
    VisID     = InfoBag.VisID;
    GTPtsList = GTPts(VisID(StructureBag.ValidPtsID),:);
    InitError = sum( sqrt ( sum( (StructureBag.Pts3D - GTPtsList).^2,2) ) );   
end