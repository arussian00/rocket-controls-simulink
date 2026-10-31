function mc_print_dispersions(spec)
%MC_PRINT_DISPERSIONS  Print 1-sigma and 3-sigma ranges.
if nargin < 1 || isempty(spec)
    spec = mc_dispersions();
end
fprintf('\nDispersion table (z clipped to +/-3, then to each row min/max)\n');
fprintf('%-16s %-8s %10s %12s %14s  %s\n', ...
    'Parameter', 'Mode', 'Nominal', '1-sigma', '3-sigma', 'Source');
for k = 1:numel(spec)
    row = spec(k);
    if row.enabled
        flag = '';
    else
        flag = ' (off)';
    end
    fprintf('%-16s %-8s %10.4g %12.4g %14.4g  %s%s\n', ...
        char(row.name), char(row.mode), row.nominal, row.sigma, 3 * row.sigma, ...
        char(row.source), flag);
end
fprintf(['Scale rows: value = nominal * (1 + sigma * z). ' ...
    'Offset rows: value = nominal + sigma * z.\n' ...
    'sensorNoise is the 1-sigma intensity in degrees, not a random scale.\n' ...
    '3-sigma is the half-range 3*sigma. A scale of 0.02 is +/-6%% at 3 sigma.\n\n']);
end
