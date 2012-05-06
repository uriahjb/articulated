%{
    A quick bit of code to test out the methods described in 
    Dynamic Simulation of Articulated Rigid Bodies with Contact 
    and Collision
%}

addpath( './utils' )
close all;

global pbody cbody dt

figure(1)
sz = 5;
axis([-sz sz -sz sz -sz sz]);

mass = 1;
inertia = eye(3);

% pos, quaternion orientation
pstate = [0 0 0 1 0 0 0 ];
cstate = [1.0 1.0 1.0 1 0 0 0 ];
% pos, angular rates
dstate = [0 0 0 0 0 0];

% plot the joint position
% xtarget? qjoint?
qjoint = [1 0 0 0];
xtarget = [0 0 0];
qtarget = [1 0 0 0];
%qtarget = RotToQuat( rotx(pi/100) )';


pbody = mk_cube( pstate, dstate, mass, inertia, [1 0 0] );
pbody.jx = [0.5 0.5 0.5];
pbody.jq = qjoint;
pbody.xtarget = xtarget;
pbody.qtarget = qtarget;
pbody.impulses = [];

cbody = mk_cube( cstate, dstate, mass, inertia, [0 0 1] );
cbody.jx = [-0.5 -0.5 -0.5];
cbody.jq = -qjoint;
cbody.impulses = [];


jh = line;
set(jh, 'xdata', [], 'ydata', [], 'zdata', []);
set(jh, 'linestyle', 'none' );
set(jh, 'marker', '*');
set(jh, 'color', [0 0 1]);

jch = line;
set(jch, 'xdata', [], 'ydata', [], 'zdata', []);
set(jch, 'linestyle', 'none' );
set(jch, 'marker', 'o');
set(jch, 'color', [0 1 0]);

%jpos = pbody.state(1:3) + quatrotate( state(4:7) ...
%                                   , pbody.jx + quatrotate( qjoint, xtarget ) ...
%                                   );                              
draw_body( pbody );
draw_body( cbody );
%set(jh, 'xdata', jpos(1), 'ydata', jpos(2), 'zdata', jpos(3));

jppos = pbody.state(1:3) + quatrotate( pbody.state(4:7) ...
                                   , pbody.jx + quatrotate( qjoint, xtarget ) ...                                   
                                   );         
jcpos = cbody.state(1:3) + quatrotate( cbody.state(4:7) ...
                               , cbody.jx ...
                               );    
jmpos = jppos + (jcpos - jppos)/2;
jpos = [jppos; jcpos];
set(jh, 'xdata', jpos(:,1), 'ydata', jpos(:,2), 'zdata', jpos(:,3));
set(jch, 'xdata', jmpos(:,1), 'ydata', jmpos(:,2), 'zdata', jmpos(:,3));


% Initial impulses
cbody = queue_impulse( cbody, [ 0 5 0], [0.5 0 0] );
pbody = queue_impulse( pbody, [-5 0 0], [0 1 0] );

% Note, this architecture will need to change quite a bit
dt = 0.01;
ang = 0.0;
while true
    
    % Solve newton iteration for position
    %j = newton( @f_pos, @df_pos, [0 0 0], pbody, cbody, dt, 0.0001 );    
    
        
    %{
    max_iter = 10
    pbtmp = pbody;
    cbtmp = cbody;
    for i = 1:max_iter
        epsilon = 1/max_iter;
        %[j,fval,exitflag,output,jacobian] = fsolve(@f_pos_mod, [0 0 0]);
        %jacobian        
        %jacob( @f_pos, [0 0 0], pbody, cbody, dt )  
        %j   
        j = newton( @f_pos, @df_pos, [0 0 0], pbody, cbody, dt, epsilon )
        j = epsilon.*j;
        jposp = jpos_p( pbody );
        jposc = jpos_c( cbody );    
        pbody.dstate(1:3) = pbody.dstate(1:3) + j./pbody.mass;
        pbody.dstate(4:6) = pbody.dstate(4:6) ...
                         + (pbody.inertia\cross( jposp, j )')';
        cbody.dstate(1:3) = cbody.dstate(1:3) - j./cbody.mass;
        cbody.dstate(4:6) = cbody.dstate(4:6) ...
                         - (cbody.inertia\cross( jposc, j )')';
        pbody = integrate_pos( pbody, dt );
        cbody = integrate_pos( cbody, dt );
        


        %[jt,fval,exitflag,output,jacobian] = fsolve(@f_orientation_mod, [0 0 0]);
        %jacobian
        %jacob( @f_orientation, [0 0 0], pbody, cbody, dt )'    
        %jt
        
        jt = newton( @f_orientation, @df_pos, [0 0 0], pbody, cbody, dt, epsilon )
        jt = epsilon.*jt;
        jposp = jpos_p( pbody );
        jposc = jpos_c( cbody );
        pbody.dstate(1:3) = pbody.dstate(1:3);
        pbody.dstate(4:6) = pbody.dstate(4:6) ...
                         + (pbody.inertia\cross( jposp, j )')';
        cbody.dstate(1:3) = cbody.dstate(1:3);
        cbody.dstate(4:6) = cbody.dstate(4:6) ...
                         - (cbody.inertia\cross( jposc, j )')';
        pbody = integrate_pos( pbody, dt );
        cbody = integrate_pos( cbody, dt );             
        
        
        draw_body( pbody );
        draw_body( cbody );
        drawnow
    
    end        
    %}    
    
    % We need to find jx and jq for each joint, and from there find, 
    % xtarget, jtarget

        
    %cbody = queue_impulse( cbody, [0 0 4.9*dt], [1 0.5 0] );
    %pbody = queue_impulse( pbody, [0 0 -4.9*dt], [0 0 0] );

    pbody = integrate_vel( pbody );
    cbody = integrate_vel( cbody );             
       
    vtype = [1 1 1];
    wtype = [1 1 1];
    
    jposp = jpos_p( pbody );
    jposc = jpos_c( cbody );
    
    [j,jt] = vel_stabilize( pbody, cbody, vtype, wtype);
    pbody.dstate(1:3) = pbody.dstate(1:3) + j./pbody.mass;
    pbody.dstate(4:6) = pbody.dstate(4:6) ...
                     + (pbody.inertia\cross( jposp, j )')';
    cbody.dstate(1:3) = cbody.dstate(1:3) - j./cbody.mass;
    cbody.dstate(4:6) = cbody.dstate(4:6) ...
                     - (cbody.inertia\cross( jposc, j )')';
    pbody.dstate(1:3) = pbody.dstate(1:3);
    pbody.dstate(4:6) = pbody.dstate(4:6) ...
                      + (pbody.inertia\jt')';
    cbody.dstate(1:3) = cbody.dstate(1:3);
    cbody.dstate(4:6) = cbody.dstate(4:6) ...
                      - (cbody.inertia\jt')';        
                  
    [xnew, qnew] = joint_evolve( pbody, cbody, dt );    
    [xtarget, qtarget] = point_constraint( xnew, qnew );
    %xtarget = xnew;
    %qtarget = qnew;
    pbody.xtarget = xtarget;
    pbody.qtarget = qtarget;
    
    jposp = jpos_p( pbody );
    jposc = jpos_c( cbody );
        
    %[j,fval,exitflag,output,jacobian] = fsolve(@f_pos_mod, [0 0 0]);    
    %jacobian
    %jacob( @f_pos, [0 0 0], pbody, cbody, dt )
    j = newton( @f_pos, @df_pos, [0 0 0], pbody, cbody, dt, 0.00001 );
        
    pbody.dstate(1:3) = pbody.dstate(1:3) + j./pbody.mass;
    pbody.dstate(4:6) = pbody.dstate(4:6) ...
                     + (pbody.inertia\cross( jposp, j )')';
    cbody.dstate(1:3) = cbody.dstate(1:3) - j./cbody.mass;
    cbody.dstate(4:6) = cbody.dstate(4:6) ...
                     - (cbody.inertia\cross( jposc, j )')';

    %[jt,fval,exitflag,output,jacobian] = fsolve(@f_orientation_mod, [0 0 0]);
    %jacobian
    %jacob( @f_orientation, [0 0 0], pbody, cbody, dt )'
    
    jt = newton( @f_orientation, @df_pos, [0 0 0], pbody, cbody, dt, 0.00001 );  
    jt
    
    pbody.dstate(1:3) = pbody.dstate(1:3);
    pbody.dstate(4:6) = pbody.dstate(4:6) ...
                      + (pbody.inertia\jt')';
    cbody.dstate(1:3) = cbody.dstate(1:3);
    cbody.dstate(4:6) = cbody.dstate(4:6) ...
                      - (cbody.inertia\jt')';                                     
                  
    pbody = integrate_pos( pbody, dt );
    cbody = integrate_pos( cbody, dt );  
    
    jposp = jpos_p( pbody );
    jposc = jpos_c( cbody );
    
    [j,jt] = vel_stabilize( pbody, cbody, vtype, wtype);
    pbody.dstate(1:3) = pbody.dstate(1:3) + j./pbody.mass;
    pbody.dstate(4:6) = pbody.dstate(4:6) ...
                     + (pbody.inertia\cross( jposp, j )')';
    cbody.dstate(1:3) = cbody.dstate(1:3) - j./cbody.mass;
    cbody.dstate(4:6) = cbody.dstate(4:6) ...
                     - (cbody.inertia\cross( jposc, j )')';
    pbody.dstate(1:3) = pbody.dstate(1:3);
    pbody.dstate(4:6) = pbody.dstate(4:6) ...
                      + (pbody.inertia\jt')';
    cbody.dstate(1:3) = cbody.dstate(1:3);
    cbody.dstate(4:6) = cbody.dstate(4:6) ...
                      - (cbody.inertia\jt')';
    
    % Evolve joint position forward
    %{
    [xnew, qnew] = joint_evolve( pbody, cbody );    
    qnew
    pbody.xtarget = xnew;
    pbody.qtarget = qnew;
    %}
    
    
    draw_body( pbody );
    draw_body( cbody );
    
    jppos = pbody.state(1:3) + quatrotate( pbody.state(4:7) ...
                                   , pbody.jx + quatrotate( qjoint, xtarget ) ...                                   
                                   );         
    jcpos = cbody.state(1:3) + quatrotate( cbody.state(4:7) ...
                                   , cbody.jx ...
                                   );    
    jmpos = jppos + (jcpos - jppos)/2;
    jpos = [jppos; jcpos];
    set(jh, 'xdata', jpos(:,1), 'ydata', jpos(:,2), 'zdata', jpos(:,3));
    set(jch, 'xdata', jmpos(:,1), 'ydata', jmpos(:,2), 'zdata', jmpos(:,3));

    % lets draw the 
    drawnow
    
end


%{
dt = 0.01;
while true
    
    % 1. Integrate velocities
    
    for i = 1:length(bodies)
      bodies{i} = integrate_vel( bodies{i}, [0.1 0.0 0.0], [0 1.0 1.0]);
    end
    
    
    % 2. Velocity post stabilization
    %    - this is something important
    
    % 3. Articulation pre-stabilization
    
    % 4. Integrate positions
    for i = 1:length(bodies)
        bodies{i} = integrate_pos( bodies{i}, dt );
    end
    
    % 5. Velocity post-stabilization
    
    % Apply transform and draw
    for i = 1:length(bodies)
        draw_body( bodies{i} );
    end
    drawnow        
    
end
%}
    