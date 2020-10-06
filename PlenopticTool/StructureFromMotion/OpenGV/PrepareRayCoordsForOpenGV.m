% function: prepare data used for OpenGV to calculate the relative poses
% between two LF frames.
function [raysRef, raysCom, rays1Ids, rays2Ids ] = PrepareRayCoordsForOpenGV(RayCellMatched_Ref,RayCellMatched_Com)
    MatchedFeatureNum  = size(RayCellMatched_Ref,1);
    TotalRayNum    = 0;
    if MatchedFeatureNum>0
        for k=1:MatchedFeatureNum
            RayArrayRef = RayCellMatched_Ref{k};
            RayArrayCom = RayCellMatched_Com{k};
            NumRayRef   = size(RayArrayRef,1);
            NumRayCom   = size(RayArrayCom,1);
            TotalRayNum = TotalRayNum + NumRayRef*NumRayCom;
        end
    end
    rays1Ids = zeros(TotalRayNum,2);
    rays2Ids = zeros(TotalRayNum,2);
    raysRef    = zeros(TotalRayNum,6); 
    raysCom    = zeros(TotalRayNum,6);
    idL = 0;
    idR = 0;
    for i = 1:MatchedFeatureNum  % 遍历初始两帧所有的匹配特征。
        RayArrayRef = RayCellMatched_Ref{i};
        RayArrayCom = RayCellMatched_Com{i};
        numRaysPerPointL   = size(RayArrayRef,1);
        numRaysPerPointR   = size(RayArrayCom,1);

        for j = 1:numRaysPerPointL  % 对应前一帧 当前特征对应的 特征光线数量。
            % each feature in the left light field, match it with all the
            % fearures from the right light field.
            raysRef(idL+1:idL+numRaysPerPointR, :) = repmat(RayArrayRef(j, :), numRaysPerPointR, 1);
            tmp = repmat([i j], numRaysPerPointR, 1);
            rays1Ids(idL+1:idL+numRaysPerPointR, :) = tmp;        
            raysCom(idR+1:idR+numRaysPerPointR, :) = RayArrayCom;
            tmp = [i*ones(numRaysPerPointR, 1) (1:numRaysPerPointR)'];
            rays2Ids(idR+1:idR+numRaysPerPointR, :) = tmp;        
            idL = idL + numRaysPerPointR;
            idR = idR + numRaysPerPointR;        
        end
    end

end
