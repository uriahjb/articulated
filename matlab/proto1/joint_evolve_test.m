%{
    getting the evolved joint state
%}
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

xtarget = quatrotate( quatinv(q_pj), ( quatrotate( quatinv(q_wp), ( x_wc + quatrotate( q_wc, x_cj ) - x_wp )) - x_pj ) );
qtarget = quatmultiply( quatinv(q_pj), quatmultiply( quatinv(q_wp), quatmultiply( q_wc, q_cj )));

