function jacob = df_orientation( j, pbody, cbody, dt )
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
  
    r_p = quatrotate( q_wp, x_pj + quatrotate( q_pj, xtarget ));
    r_c = quatrotate( q_wc, x_cj);
    
    j_xp = x_wp + quatrotate( q_wp ...
                            , x_pj + quatrotate( q_pj, xtarget ) ...                                   
                            );
    j_xc = x_wc + quatrotate( q_wc ...
                            , x_cj ...                                   
                            );                            
        
    int_omega_p = omega_wp.*dt + dt.*(p_inertia\cross( j_xp, j )')';
    %Don't forget this minus sign !!!!
    int_omega_c = omega_wc.*dt - dt.*(c_inertia\cross( j_xc, -j)')';
    
    w_p = norm(int_omega_p)/2;
    theta_p = int_omega_p./norm(int_omega_p);
    w_c = norm(int_omega_p)/2;
    theta_c = int_omega_p./norm(int_omega_p);
    
    thetadotp = (dt/2).*w_p*inv(p_inertia);
    wdotp = (dt/2).*(eye(3) - w_p*w_p')*inv(p_inertia);
    thetadotc = -(dt/2).*w_c*inv(c_inertia)
    wdotc = -(dt/2).*(eye(3) - w_c*w_c')*inv(c_inertia)
    
    qp = quatmultiply( q_wp, quatmultiply( q_pj, quatmultiply( qtarget )));
    qc = quatmultiply( q_wc, q_cj );
       
    dAdjp = -qp(1).*thetadotp*sin(theta_p) ...
          - dot(qp(2:4), w_p).*thetadotp*cos(theta_p) ...
          - dot(qp(2:4), wdot_p)*sin(theta);
    dBdjp = 
    
    jacob = [];
end
    
    
    