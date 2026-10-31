function value = mc_get(draw, name, fallback)
%MC_GET  Read one dispersion knob, including reserved names.
%
%   value = mc_get(draw, "lr_blend", 0)
%
%   Later steps can read a knob that mc_realize stored on plant.knobs
%   without changing the sampler.

if isstruct(draw) && isfield(draw, 'knobs')
    draw = draw.knobs;
end
name = char(name);
if isstruct(draw) && isfield(draw, name)
    value = draw.(name);
else
    value = fallback;
end
end
