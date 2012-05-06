%{
    Evolve the joint position forward, 
    the idea here is to look forward and make a
    guess at the desired joint state, Jnew, 
    then we use a 'black-box' joint model to 
    modify Jnew to Jtarget for our 
    nonlinear solver
%}
function [xnew, qnew] = joint_evolve( pbody, cbody, dt )
    x_wp = pbody.state(1:3);
    q_wp = pbody.state(4:7);
    v_wp = pbody.dstate(1:3);
    omega_wp = pbody.dstate(4:6);
    m_p = pbody.mass;    
    x_pj = pbody.jx;
    q_pj = pbody.jq;
    xtarget = pbody.xtarget;
    p_inertia = pbody.inertia;

    x_wc = cbody.state(1:3);
    q_wc = cbody.state(4:7);
    v_wc = cbody.dstate(1:3);
    omega_wc = cbody.dstate(4:6);
    m_c = cbody.mass;    
    x_cj = cbody.jx;
    q_cj = cbody.jq;
    c_inertia = cbody.inertia;   
    
    q_hp = q_hat( omega_wp.*dt );    
    q_wp1 = quatmultiply( q_hp, q_wp );
    
    q_hc = q_hat( omega_wc.*dt );    
    q_wc1 = quatmultiply( q_hc, q_wc );
       
    x_wp1 = x_wp + v_wp.*dt;
    x_wc1 = x_wc + v_wc.*dt;
    
    xnew = quatrotate( quatinv(q_pj), ( quatrotate( quatinv(q_wp1), ( x_wc1 + quatrotate( q_wc1, x_cj ) - x_wp1 )) - x_pj ) );   
    qnew = quatmultiply( quatinv(q_pj), quatmultiply( quatinv(q_wp1), quatmultiply( q_wc1, q_cj )));       
    
    %xnew = quatrotate( quatinv(q_pj), ( quatrotate( quatinv(q_wp), ( x_wc + quatrotate( q_wc, x_cj ) - x_wp )) - x_pj ) );   
    %qnew = quatmultiply( quatinv(q_pj), quatmultiply( quatinv(q_wp), quatmultiply( q_wc, q_cj )));       
    
end
