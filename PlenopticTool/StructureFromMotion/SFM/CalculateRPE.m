function ReproErrors = CalculateRPE(CurPtsCamCoords, FeatureInfoCell, InfoBag)


    GridCoords        = InfoBag.GridCoords;
    GridCoordsX       = GridCoords(:,:,1);
    GridCoordsY       = GridCoords(:,:,2);
    NumSeenPts        = size(FeatureInfoCell,1);
    ReproErrors       = zeros(NumSeenPts,1);
    for j = 1:NumSeenPts                      % go through all matched features.
        Cur_FeatureInfo = FeatureInfoCell{j};
        Cur_PtsLensID = Cur_FeatureInfo(:,3);
        ucs           = GridCoordsX(Cur_PtsLensID);                                             % Î¢Í¸¾µÖÐÐÄXÏñËØ×ø±ê¡£
        vcs           = GridCoordsY(Cur_PtsLensID);
        du            = InfoBag.fx * CurPtsCamCoords(1, j) - CurPtsCamCoords(3, j).*(ucs - InfoBag.cx);
        dv            = InfoBag.fy * CurPtsCamCoords(2, j) - CurPtsCamCoords(3, j).*(vcs - InfoBag.cy);
        nominator     = 1./(InfoBag.K1 * CurPtsCamCoords(3, j) + InfoBag.K2);
        du            = nominator.*du;
        dv            = nominator.*dv;
        us            = ucs+du;
        vs            = vcs+dv;
        err           = [us,vs]-Cur_FeatureInfo(:,1:2);   % calculate reprojection error.
        err           = mean( sqrt( sum( err.^2 , 2 ) ) );
        ReproErrors(j)= err;
    end

end