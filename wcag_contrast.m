function ratio = wcag_contrast(a, b)
%WCAG_CONTRAST  WCAG 2 contrast ratio of two opaque colors.
%
%   ratio = wcag_contrast(a, b)
%
%   1 is no contrast. 21 is black on white. Normal text needs 4.5.

a = color_rows(a);
b = color_rows(b);
if isempty(a) || isempty(b)
    ratio = NaN;
    return
end
la = rel_luminance(a(1, :));
lb = rel_luminance(b(1, :));
ratio = (max(la, lb) + 0.05) / (min(la, lb) + 0.05);
end


function lum = rel_luminance(rgb)
rgb = double(rgb);
lin = zeros(1, 3);
for k = 1:3
    c = rgb(k);
    if c <= 0.03928
        lin(k) = c / 12.92;
    else
        lin(k) = ((c + 0.055) / 1.055) ^ 2.4;
    end
end
lum = 0.2126 * lin(1) + 0.7152 * lin(2) + 0.0722 * lin(3);
end
