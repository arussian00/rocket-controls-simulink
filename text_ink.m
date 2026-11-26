function ink = text_ink(bg)
%TEXT_INK  Near-white or near-black, whichever clears the backdrop.
%
%   ink = text_ink(bg)
%
%   bg is one RGB color or several rows (a table stripe). The choice is
%   the ink with the higher minimum WCAG ratio. Dark backdrops get
%   #F0F0F0. Light backdrops, including a light tab strip, get near-black.

theme = plot_theme();
rows = color_rows(bg);
if isempty(rows)
    rows = theme.fallback;
end
lightScore = worst_contrast(theme.lightInk, rows);
darkScore = worst_contrast(theme.darkInk, rows);
if darkScore > lightScore
    ink = theme.darkInk;
elseif lightScore > darkScore
    ink = theme.lightInk;
elseif mean(rows(:)) >= 0.5
    ink = theme.darkInk;
else
    ink = theme.lightInk;
end
end


function score = worst_contrast(ink, rows)
score = inf;
for r = 1:size(rows, 1)
    score = min(score, wcag_contrast(ink, rows(r, :)));
end
end
