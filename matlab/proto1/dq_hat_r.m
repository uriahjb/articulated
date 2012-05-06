% Descrete derivative of quaternion multiplied with vector ... this stuff is 
% kinda a pain
function val = dq_hat_r( omega, r, inertia, dt )
   
    if norm(omega) == 0
        val = [0 0 0];
        return;
    end
    
    theta = norm(omega)/2;
    w = omega/norm(omega);
    
    % I think i need to permute all of these +/- thetadot +/- wdot
    % since they are wrapped in a norm
    sign = 1;
    thetadot = sign*(dt/2)*w*inv(inertia)*r';
    wdot = sign*((dt/2.*theta)*(eye(3) - w'*w)*inv(inertia)*r')';
    
    val = - 4*r*thetadot*cos(theta)*sin(theta) ...
          - 2*cross(r,w)*thetadot*cos(theta)*cos(theta) ...
          + 2*cross(r,w)*thetadot*sin(theta)*sin(theta) ...
          - 2*cross(r,wdot)*cos(theta)*sin(theta) ...
          + 4*dot(r,w)*w*thetadot*sin(theta)*cos(theta) ...
          + 2*w*r'*wdot*sin(theta)*sin(theta) ...
          + 2*dot(r,w)*wdot*sin(theta)*sin(theta);            
end
    