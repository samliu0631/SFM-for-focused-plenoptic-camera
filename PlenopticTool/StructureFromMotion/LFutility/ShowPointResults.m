function ShowPointResults(corn_est)
    FrameNum                     = size(corn_est,1);
    for j=1:FrameNum
        corn_current                = corn_est{j};
        CornerNum                   = size( corn_current,1);
        figure; 
        hold on;
        if CornerNum>0
            for k=1:CornerNum
                cornercoords        = corn_current{k};
                if ~isempty(cornercoords)
                    plot(cornercoords(:,1),cornercoords(:,2),'*');
                end
            end
        end
        hold off;
    end
end