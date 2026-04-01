function [M_out] = rotate2world(M, phi, Ow)
    % ROTATE2WORLD Rotate {x,y} coordinates in M by the angle phi around origin Ow
    %   M       struct {x, y, ...}
    %   phi     double
    %   Ow      double(2)    center of rotation (x, y)

    x0 = Ow(1);
    y0 = Ow(2);

    % Transform corrdinates in Ow
    X = [M.x - x0; M.y - y0];

    % 2D rotation matrix
    R = @(psi) [cos(psi), sin(psi); -sin(psi), cos(psi)];

    % Rotate coordiantes
    X_ = R(phi) * X;

    % copy struct and asign rotated coordinates
    M_out = M;
    M_out.x = X_(1, :) + x0;
    M_out.y = X_(2, :) + y0;
    M_out.yaw = M.yaw - phi;

end
