function theme = plot_theme()
%PLOT_THEME  Shared colors for every figure this project draws.
%
%   theme = plot_theme()
%
%   bg is the dark axes interior. grid is the grey drawn on that interior.
%   lightInk (#F0F0F0) is for dark backdrops. darkInk is for light ones,
%   including the uitab strip, which has no color property and stays light.
%   minContrast is the WCAG contrast ratio text must clear. Legends are
%   not part of the theme.

theme.lightInk = [240 240 240] / 255;
theme.darkInk = [0.05 0.05 0.05];
theme.bg = [0.07 0.08 0.10];
theme.grid = [0.78 0.81 0.85];
theme.gridAlpha = 0.35;
theme.minContrast = 4.5;
theme.tabStrip = [0.94 0.94 0.94];
theme.fallback = [0.94 0.94 0.94];
end
