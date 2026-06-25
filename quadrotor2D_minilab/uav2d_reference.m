function [r, rdot] = uav2d_reference(~, p)
% Constant position reference.

r = p.ref.pos;
rdot = [0; 0];
end
