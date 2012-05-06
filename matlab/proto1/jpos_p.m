function pos = jpos_p( pbody )

    x_wp = pbody.state(1:3);
    q_wp = pbody.state(4:7);
    v_wp = pbody.dstate(1:3);
    omega_wp = pbody.dstate(4:6);
    m_p = pbody.mass;    
    x_pj = pbody.jx;
    q_pj = pbody.jq;
    xtarget = pbody.xtarget;
    p_inertia = pbody.inertia;
    
    pos = x_wp + quatrotate( q_wp ...
                            , x_pj + quatrotate( q_pj, xtarget ) ...                                   
                            ); 
end