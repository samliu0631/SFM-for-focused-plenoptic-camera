function [pyramid, nextFrame] = SelectBestNextView(StructureBag, QueryImgInfo, AlgoConfig)
% next best view selection based on score of the image as described in the
% COLMAP paper
%
% pts3dIds      -   Ids of the already trinagulated points in the visibilyt matrix indexing.
% visMatrix     -   Number of points by number of frames, consisting of the
%                   ids of the matches in each column.
% visBool       -   Boolean matric of the visibility per point per frame
%                   (usually visBool = visMatrix ~=0).
% width         -   Width of the image.
% height        -   Height of the image.
% pyramidRes    -   Resolution of the visibility pyramid.
% queryImages   -   The ids of the images from which the next view will be
%                   selected.
% frames        -   1:numberLfFrames
% centFs        -   The central keypoints, cell array where each cell holds
%                   the keypoints of each central sub-aperture image.

% Data Input.
pts3dIds                 = StructureBag.ValidPtsID;
visMatrix                = StructureBag.visMatrix_est;
visBool                  = StructureBag.visBool_est;

queryImages              = QueryImgInfo.queryImgID;
frames                   = QueryImgInfo.framesID;
centFs                   = QueryImgInfo.PDFCell;
width                    = QueryImgInfo.width;
height                   = QueryImgInfo.height;

pyramidRes               = AlgoConfig.pyramidRes;


lvls = 2.^(1:pyramidRes) + 1;  % the number of grids.
% we can index each cell just by taking the divisiom between the step of
% the linspace and the corresponding keypoint index.
stepw = zeros(pyramidRes,1);
steph = zeros(pyramidRes,1);
for i = 1:pyramidRes
   
    ws = linspace(1, width, lvls(i));  % assign the grid.
    hs = linspace(1, height, lvls(i));
    
    stepw(i) = ws(2) - ws(1);   % the width of each grid.
    steph(i) = hs(2) - hs(1);   % the height of each grid.
    
end

pyramid = cell(numel(frames),1);
for n = 1:numel(frames)  % iterate all frames
   for i = 1:pyramidRes   % iterate all level.
      
       pyramid{n}.cells{i} = zeros(lvls(i)-1);  % store the distribution of all points.
       pyramid{n}.stepw(i) = stepw(i);          % width of the grid
       pyramid{n}.steph(i) = steph(i);          % height of the grid
       pyramid{n}.score(i) = lvls(i)-1;         % score of each level.
       pyramid{n}.S = 0;                        % total score.
       pyramid{n}.frame = n;                    % frame ID.
       
   end  
end


scores = zeros(numel(frames), 1);  % store the score of each frames.


for i = 1:size(queryImages)   % iterate all the remaining img.
    qid = queryImages(i);
    
    seeIds = find(visBool(pts3dIds, qid));  % find the ID within the visbool of the matched feature in current frame.
    
    if isempty(seeIds)
        continue;
    end
    
    seeIds = pts3dIds(seeIds);  % the id of 3d points seen in current frame¡£
    
    
    fs = centFs{qid}(1:2, visMatrix(seeIds, qid));  % the plenoptic disc center of the current frames¡£
    
    numFs = size(fs, 2);
    
    for n = 1:numFs
        for res = 1:pyramidRes
            c1 = floor(fs(1, n) / pyramid{qid}.steph(res)) + 1;
            c2 = floor(fs(2, n) / pyramid{qid}.stepw(res)) + 1;
            
            if (c1 > size(pyramid{qid}.cells{res}, 1)) % set the max value of c1.
                c1 = size(pyramid{qid}.cells{res}, 1);
            end
            
            
            if (c2 > size(pyramid{qid}.cells{res}, 1)) % set the max value of c2.
                c2 = size(pyramid{qid}.cells{res}, 1);
            end
            
            if ( c2<=0 ) || ( c1<=0 )   % If the PDF center is out of the frame size.
                 continue;
            end           
                        
            pyramid{qid}.cells{res}(c2, c1) = 1; % the maximum value of each grid is 1.
        end
    end
    
    
    for res = 1:pyramidRes  % calculate the score for each frames.
       pyramid{qid}.S = numel(find(pyramid{qid}.cells{res}))*pyramid{qid}.score(res) + pyramid{qid}.S;       
    end
    
    scores(qid) = pyramid{qid}.S;
    
end

[val, maxId] = max(scores);

if (val==0)
    nextFrame = NaN;
else

    nextFrame = pyramid{maxId}.frame;
end




end

