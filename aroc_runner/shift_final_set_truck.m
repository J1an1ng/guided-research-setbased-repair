function Rfin = shift_final_set_truck(Rfin)
    % shift the set to the origin of the position space
    c = center(Rfin);
    Rfin = zonotope(Rfin + (-c));

    % rotate the set by the orientation of the previous maneuver
    phi = -c(2);
    T = blkdiag(eye(4), [cos(phi) -sin(phi); sin(phi) cos(phi)]);

    Rfin = T * Rfin;
end
