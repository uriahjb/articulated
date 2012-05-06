%{
Integrate velocities using applied impulse 
%}
function body = integrate_vel( body )    
    i_sz = size(body.impulses);
    for i = 1:i_sz(1)    
        impulse = body.impulses(i,1:3);
        loc = body.impulses(i,4:6);
        body.dstate(1:3) = body.dstate(1:3) + impulse./body.mass;
        body.dstate(4:6) = body.dstate(4:6) ...
                         + (body.inertia\cross( loc, impulse )')';
    end
    body.impulses = [];
end
    