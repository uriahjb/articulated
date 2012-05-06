addpath( './utils' )
clear all;
close all;

figure(1)
sz = 5;
axis([-sz sz -sz sz -sz sz]);

pstate = [0 0 0 1 0 0 0];
cstate = [1 1 1 1 0 0 0];

dstate = zeros(6,1);
mass = 1;
inertia = eye(3);

jx = [0.5 0.5 0.5];
jq = [1 0 0 0];

pbdy = body( pstate, dstate, mass, inertia, [1 0 0] );
cbdy = body( cstate, dstate, mass, inertia, [0 0 1] );

utype = [1 1 1];
wtype = [1 1 1];
jnt = joint( pbdy, cbdy, utype, wtype, jx, -jx, jq, -jq );

pbdy.draw();
cbdy.draw();
drawnow();

dt = 0.01;
while true
    pbdy.integrate_velocity();
    pbdy.integrate_velocity();
    
    jnt.stabilize_velocity();
    
    j = newton( @f_pos, @df_pos, [0 0 0], pbody, cbody, dt, 0.00001 );
    jnt.apply_impulse( j );
    
    jt = newton( @f_orientation, @df_pos, [0 0 0], pbody, cbody, dt, 0.00001 );  
    jnt.apply_impulse( jt );
    
    pbdy.integrate_position();
    pbdy.integrate_position();
    
    jnt.stabilize_velocity();
    
    pbdy.draw();
    cbdy.draw();
    drawnow
end


    