function [matches, geomVer] = MatchAcrossImages( PDFCell, DescCell, InterMatchConfig)
    matchThresh   = InterMatchConfig.matchThresh;
    essHomRatio   = InterMatchConfig.essHomRatio;
    filterMatches = InterMatchConfig.filterMatches;
    K             = InterMatchConfig.K;
    ransacTh      = 1e-5;
    sigma         = 0.8;    
    numframes     = size( PDFCell,1);
    geomVer       = zeros(numframes);
    match         = cell(1, numframes-1);  % 计算两帧之间的匹配
    for i =1:(numframes-1)
        match{i}= cell(1, numframes);     
    end 
    
    if isfield(InterMatchConfig,'SFMWindowSize')
        WindowSize = InterMatchConfig.SFMWindowSize;
    else
        WindowSize = numframes;
    end
    % Prepare for the parallel computation.
    IndexXY  = zeros( numframes * (numframes-1) / 2 , 2 );
    count    = 1;
    for i = 1 : numframes-1        
        for j = i+1 : numframes
            IndexXY(count,:) = [i,j]; 
            count = count+1;
        end        
    end      
    
    % Start parallel computation.
    FramePairNum   = numframes * (numframes-1) / 2;
    IndexX         = IndexXY(:,1);
    IndexY         = IndexXY(:,2);
    matchVector    = cell(FramePairNum,1);
    geoVerVector   = zeros(FramePairNum,1);
    
    for  ii = 1 : FramePairNum  % if there is multi-core cpu, the parfor can be used.
    %parfor  ii = 1 : FramePairNum
        f1 = IndexX(ii);
        f2 = IndexY(ii);   
        if f2 > ( f1 + WindowSize )
            continue;
        end
        fprintf('f1=%d ,f2=%d\n',f1,f2);
        
        % Feature matching.
        ids  = vl_ubcmatch(DescCell{f1}', DescCell{f2}', matchThresh);  % 前一帧和后二帧 中心视角特征 做匹配
        matchVector{ii} = ids;          
   
        if (filterMatches)
            matches1 = PDFCell{f1}(ids(1, :), 1:2)'; % 提取前一帧 中心视角匹配特征的 坐标
            matches2 = PDFCell{f2}(ids(2, :), 1:2)'; % 提取后一阵 中心视角匹配特征的 坐标
            % check that you have enough points to get an estimate of the
            % essential matrix
            if (size(matches1,2) < 25) % if the matched number is smaller than 25.
                matchVector{ii} = [];
                continue;
            end
            [~, inle] = ransacFitEssentialHelper(matches1, matches2, K, K, ransacTh); % 估计两帧图像之间的基本矩阵
            [H, ~] = ransacHomographyHelper(matches1, matches2, ransacTh*10^3);
            [F, ~] = estimateFundamentalMatrix(matches1', matches2');
            [model, ~, ~] = gric_modsel(matches1, matches2, F, H, sigma); % 选择是单应模型 还是对极几何模型。
            geoVerVector (ii) = model;  % 判断两针之间是单应变换还是 对极变换
            tmp = ids;
            matchVector{ii} = zeros(2, numel(inle));
            matchVector{ii}(1, :) = tmp(1, inle);
            matchVector{ii}(2, :) = tmp(2, inle); % 只记录符合对极几何模型的内点。
        end
    end
    
    % assemble the cell match according to the IndexXY.
    for k =1 : FramePairNum
       f1 = IndexX(k);
       f2 = IndexY(k); 
       match{f1}{f2} = matchVector{k};    
       geomVer(f1, f2) = geoVerVector(k);
    end    
    matches = match;
    geomVer(geomVer == 2) = 0; % 将不符合对极几何关系的帧之间的关系置为0
end