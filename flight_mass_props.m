function [mp, bendWn, bendHz] = flight_mass_props(flight, V)
%FLIGHT_MASS_PROPS  Mass properties for one design-point flight struct.
%
%   [mp, bendWn, bendHz] = flight_mass_props(flight, V)
%
%   design_pitch_controller copies each flight and then stores Control
%   System Toolbox objects on it. After that copy, MATLAB Online can
%   return a D.flight element that still has the scalar record (mass,
%   thrust, q, length, mu, lambda, arm, inertia) but no nested massProps
%   struct. Those scalars are the contract. This rebuilds the tank model
%   from them when the nested struct is absent.
%
%   The centre-of-pressure reference is V.staticMarginAscent or
%   V.staticMarginLanding, the same knobs make_flight used. It is not
%   flight.staticMarginCal: that field is overwritten with the realized
%   margin after the CG has moved, and using it again would move the CP.

if nargin < 2
    error('flight_mass_props:NeedV', 'Pass the vehicle_params struct as the second argument.');
end
if ~has_number(flight, 'mass') || ~has_number(flight, 'thrust') || ~has_number(flight, 'q')
    error('flight_mass_props:Record', ...
        ['Flight "%s" has no mass/thrust/q record. Fields: %s'], ...
        flight_label(flight), field_list(flight));
end

[config, refSm] = flight_config(flight, V);
if has_mass_props(flight)
    mp = flight.massProps;
else
    mp = vehicle_mass_properties(flight.mass, flight.thrust, flight.q, config, V, refSm);
end

if isfield(flight, 'bendWn') && ~isempty(flight.bendWn)
    bendWn = flight.bendWn(:).';
    if isfield(flight, 'bendHz') && ~isempty(flight.bendHz)
        bendHz = flight.bendHz(:).';
    else
        bendHz = bendWn / (2 * pi);
    end
else
    bd = bending_model(mp, V);
    bendWn = [bd.wn];
    bendHz = [bd.hz];
end
end


function [config, refSm] = flight_config(flight, V)
if isfield(flight, 'config') && ~isempty(flight.config)
    config = string(flight.config);
elseif has_number(flight, 'length') && flight.length >= V.stackLength - 1
    config = "stack";
else
    config = "booster";
end
if strcmp(config, "stack")
    refSm = V.staticMarginAscent;
else
    refSm = V.staticMarginLanding;
end
end


function ok = has_mass_props(flight)
ok = isfield(flight, 'massProps') && isstruct(flight.massProps) ...
    && isfield(flight.massProps, 'mu') && isfield(flight.massProps, 'I') ...
    && ~isempty(flight.massProps.mu);
end


function ok = has_number(flight, name)
ok = isfield(flight, name) && isnumeric(flight.(name)) && ~isempty(flight.(name));
end


function label = flight_label(flight)
if isfield(flight, 'name') && ~isempty(flight.name)
    label = char(string(flight.name));
else
    label = '?';
end
end


function text = field_list(flight)
names = fieldnames(flight);
if isempty(names)
    text = '(none)';
else
    text = strjoin(names(:).', ', ');
end
end
