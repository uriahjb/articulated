function [j, jt] = vel_stabilize( pbody, cbody, utype, wtype )
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
    c_inertia = cbody.inertia;


    j_xp = x_wp + quatrotate( q_wp ...
                            , x_pj + quatrotate( q_pj, xtarget ) ...                                   
                            ); 

    j_xc = x_wc + quatrotate( q_wc ...
                            , x_cj ...                                   
                            );

    jA1 = (1/m_p + 1/m_c).*eye(3) ...
          + skew(j_xp)'*inv(p_inertia)*skew(j_xp) ...
          + skew(j_xc)'*inv(c_inertia)*skew(j_xc);
    %jA1 = jA1';

    jtA1 = skew(j_xp)'*inv(p_inertia) + skew(j_xc)'*inv(c_inertia);
    %jtA1 = jtA1';
    
    jA2 = inv(p_inertia)*skew(j_xp) + inv(c_inertia)*skew(j_xc);
    %jA2 = jA2';

    jtA2 = inv(p_inertia) + inv(c_inertia);
    %jtA2 = jtA2';

    urel = v_wp - v_wc;
    unew = urel.*utype;

    wrel = omega_wp - omega_wc;
    wnew = wrel.*wtype;

    A = [jA1 jtA1; jA2 jtA2];
    js = A\[unew-urel wnew-wrel]';
    
    j = js(1:3)';
    jt = js(4:6)';
end
