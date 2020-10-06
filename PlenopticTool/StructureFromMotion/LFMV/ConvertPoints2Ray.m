function LFRayCell = ConvertPoints2Ray(Points,InfoBag)
    Intrinsic.K1 = InfoBag.K1;
    Intrinsic.K2 = InfoBag.K2;
    Intrinsic.fx = InfoBag.fx;
    Intrinsic.fy = InfoBag.fy;
    Intrinsic.GridCoords = InfoBag.GridCoords ;
    Intrinsic.cx = InfoBag.cx;
    Intrinsic.cy = InfoBag.cy;   
    LFRayCell = ConvertPoints2Ray6D(Points,Intrinsic);   
end
