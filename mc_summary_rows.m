function rows = mc_summary_rows(MC)
%MC_SUMMARY_ROWS  Pass rate and worst case for the Stats uitable.
%
%   rows = mc_summary_rows(MC)
%
%   Three rows, 15 columns, matching the pitch dashboard stats table.
%   Every cell is a char row or a numeric scalar. Blank cells are ''.

rows = blank_rows(3, 15);
rows{1, 1} = 'MC pass rate';
rows{1, 2} = sprintf('%.1f%% (%d/%d)', MC.passRate, MC.nPass, MC.N);

rows{2, 1} = 'MC worst margins';
if MC.worst.pm >= 30 && MC.worst.gm >= 6
    rows{2, 2} = 'yes';
else
    rows{2, 2} = 'no';
end
rows{2, 3} = MC.worst.pm;
rows{2, 4} = MC.worst.gm;

rows{3, 1} = 'MC worst 30 deg';
rows{3, 2} = 'worst';
rows{3, 11} = MC.worst.peakGimbal30;
rows{3, 12} = MC.worst.peakRate30;
rows{3, 13} = MC.worst.tOnAngle30;
rows{3, 14} = MC.worst.tOnRate30;
end


function rows = blank_rows(nRow, nCol)
rows = cell(nRow, nCol);
for r = 1:nRow
    for c = 1:nCol
        rows{r, c} = '';
    end
end
end
