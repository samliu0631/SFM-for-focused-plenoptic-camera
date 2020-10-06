function StructureBag = ValidateAllpointsbyRPE(StructureBag, QueryImgInfo , InfoBag)
    %FUNCTION : Validate the RPE of all PDF center with regard to 3D points.
    
    % Data input.
    visMatrix    = StructureBag.visMatrix_est;
    visBool      = StructureBag.visBool_est;
    Pts3D        = StructureBag.Pts3D;        % The 3d coordinates of reconstructed 3D points.
    PtsID        = StructureBag.ValidPtsID;   % The id of reconstructed 3D points in visMatrix.
    Poses        = StructureBag.poses;        % The camera pose of current camera.
    PDFCell      = QueryImgInfo.PDFCell;      % The pdf coordinates of all frames.
    Pts3DNum     = size(Pts3D,1);             % The number of reconstructed 3D points.
    for i = 1: Pts3DNum
        Cur3DPts   = Pts3D(i,:);
        Cur3DPtsID = PtsID(i);
        SeenCamID  = find( visBool( Cur3DPtsID, : ) );
        CamNum     = size( SeenCamID, 2 );
        for j = 1:CamNum   % iterate all camera see current  3D point.
            
            % Get the current camera id.
            CurCamID        = SeenCamID(j);                  
            
            % Convert World coordinates to camera coordinates
            if isempty(Poses{CurCamID})
                continue;                
            end   
            
            Rh              = Poses{CurCamID}(1:3,1:3);
            th              = Poses{CurCamID}(1:3,4);
            extMat          = [Rh' -Rh'*th; zeros(1, 3) 1];
            WorldPts        = [Cur3DPts, 1];
            CameraPts       = extMat*WorldPts';
            CurPtsCamCoords = CameraPts(1:3,:);
            
            % Extract the feature.
            FeatIDwithinFrame = visMatrix( Cur3DPtsID, CurCamID );
            if FeatIDwithinFrame~=0 && ~isempty(FeatIDwithinFrame) % make sure there is detected feature.
                 PDFMeasured  =  PDFCell{CurCamID}(: , FeatIDwithinFrame );    
                 RPE          =  CalculateRPEofPDF(CurPtsCamCoords,PDFMeasured, InfoBag); % add the constrain of pdr.
                 
                 if RPE > 10
                    StructureBag.visMatrix_est(Cur3DPtsID,CurCamID) = 0;
                    StructureBag.visBool_est(Cur3DPtsID,CurCamID) = 0;
                 end
            end      
        end
    end
end