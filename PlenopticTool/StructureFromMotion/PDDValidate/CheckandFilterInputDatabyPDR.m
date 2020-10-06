function [PDFCell, DescCell] = CheckandFilterInputDatabyPDR(PDFCell, DescCell,varargin)
    if nargin == 4
        MinPDR       = varargin{1};
        MaxPDR       = varargin{2};
    else
        MinPDR       = 3;
        MaxPDR       = 30;
    end
    
    FrameNum = size(PDFCell,1);
    for i = 1:FrameNum
        CurPDF   = PDFCell{i};
        CurDesc  = DescCell{i};

        % store the feature with valid pdr.
        CurPDR       = CurPDF(:,3);
        PDR_ValidID  = ( abs(CurPDR) > MinPDR )  &  ( abs(CurPDR) < MaxPDR ) ;
        CurPDF       = CurPDF(PDR_ValidID,:);
        CurDesc      = CurDesc(PDR_ValidID,:);

        % Update the GeneratedDataBag.
        PDFCell{i}   = CurPDF;
        DescCell{i}  = CurDesc ;
    end

end