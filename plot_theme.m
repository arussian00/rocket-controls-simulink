function theme = plot_theme()
%PLOT_THEME  Shared colors for every figure this project draws.
%
%   theme = plot_theme()
%
%   apply_plot_theme paints these onto a figure. bg is the dark axes
%   interior. The figure, tabs, panels, grids and tables use the same
%   color so ink (#F0F0F0) stays readable on the chrome around the axes.
%   grid is the light grey already used on that dark interior. Legends are
%   not part of the theme.

theme.ink = [240 240 240] / 255;
theme.bg = [0.07 0.08 0.10];
theme.grid = [0.78 0.81 0.85];
theme.gridAlpha = 0.35;
end
