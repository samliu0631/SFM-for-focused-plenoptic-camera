function LFRayCell = ConvertPoints2Ray6D(Points,Intrinsic)
    GridCoordsX = Intrinsic.GridCoords(:,:,1);
    GridCoordsY = Intrinsic.GridCoords(:,:,2);
    FrameNum         = size(Points,1);
    LFRayCell   =  cell(FrameNum,1);
    for j=1:FrameNum
        Pointcorn_current  = Points{j};
        PointNum           = size( Pointcorn_current,1);
        Ray6DCell     = cell(PointNum,1);
        if PointNum>0
            for k=1:PointNum
                Pointcoords        = Pointcorn_current{k};
                if ~isempty(Pointcoords)
                    PointCoordXY  = Pointcoords(:,1:2);
                    LensID  = Pointcoords(:,3);
                    raynum  = size(PointCoordXY, 1);
                    Ray_Direction = zeros(raynum,3);
                    Ray_Position = zeros(raynum,3);
                    McImgCenterX = GridCoordsX(LensID);
                    McImgCenterY = GridCoordsY(LensID);
                    McImgCenter  = [McImgCenterX,McImgCenterY];
                    Ray_Direction(:,1:2) = Intrinsic.K1.*(PointCoordXY-McImgCenter)./[Intrinsic.fx,Intrinsic.fy]+(McImgCenter-[Intrinsic.cx,Intrinsic.cy])./[Intrinsic.fx,Intrinsic.fy];

                    Ray_Direction(:,3) = ones(raynum,1);
                    Raynorm   = sqrt( sum( Ray_Direction.^2, 2 ) );
                    Ray_Direction =Ray_Direction./Raynorm;
                    
                    % position on the main lens plane
                    Ray_Position(:,1:2) = Intrinsic.K2.*(PointCoordXY-McImgCenter)./[Intrinsic.fx,Intrinsic.fy];
                    Ray_Position(:,3) = zeros(raynum,1);
                    
                    % position on the virtual sub-camera plane.
                    %K2K1= -Intrinsic.K2/Intrinsic.K1;
                    %Ray_Position(:,1:2) = K2K1.*(McImgCenter-[Intrinsic.cx,Intrinsic.cy])./[Intrinsic.fx,Intrinsic.fy];
                    %Ray_Position(:,3)   = K2K1.*ones(raynum,1);
                    
                    
                    %M = cross(Ray_Position', Ray_Direction');
                    RayDirect_Position = [Ray_Direction,Ray_Position];
                    Ray6DCell{k} = RayDirect_Position;
                end
            end
        end
        LFRayCell{j}=Ray6DCell;    
    end

end