%{
    A quick bit of code to test out the methods described in 
    Dynamic Simulation of Articulated Rigid Bodies with Contact 
    and Collision
%}

addpath( './utils' )

figure(1)
axis([-5 5 -5 5 -5 5]);

mass = 1;
inertia = eye(3);

% pos, quaternion orientation
state = [0 0 0 RotToQuat( rotx(0) )' ];
% pos, angular rates
dstate = [0 0 0 0 0 0];

body = mk_body( state, dstate, mass, inertia );

bodies = {};
bodies = [bodies body];

while true
    
    % 1. Integrate velocities
    %{
    for i = 1:length(bodies)
      integrate_vel( bodies(i) )  
    end
    %}
    
    % 2. Velocity post stabilization
    %    - this is something important
    
    % 3. Articulation pre-stabilization
    
    % 4. Integrate positions
    for i = 1:length(bodies)
        integrate_pos( bodies{i} )
    end
    
    % 5. Velocity post-stabilization
    
    % Apply transform and draw
    for i = 1:length(bodies)
        draw_body( bodies{i} )
    end
    drawnow        
    
end
    