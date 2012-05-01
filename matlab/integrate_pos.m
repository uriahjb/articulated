%{
    A simple euler integration routine
%}
function body = integrate_pos( body, dt)
    state = body.state;
    dstate = body.dstate;
    
    % Integrate position
    state(1:3) = state(1:3) + dstate(1:3).*dt;
    
    K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
    quaterror = 1 - norm(state(4:7)); 
    p = dstate(4);
    q = dstate(5);
    r = dstate(6);
    qdot = -1/2*[0,-p,-q,-r;...
                 p,0,-r,q;...
                 q,r,0,-p;...
                 r,-q,p,0]*state(4:7)' + K_quat*quaterror*state(4:7)';
         
    state(7:10) = state(7:10) + qdot.*dt;
    
    body.state = state;
end

    
    