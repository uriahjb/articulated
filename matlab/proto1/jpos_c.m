function pos = jpos_c( cbody )
    
    x_wc = cbody.state(1:3);
    q_wc = cbody.state(4:7);
    v_wc = cbody.dstate(1:3);
    omega_wc = cbody.dstate(4:6);
    m_c = cbody.mass;    
    x_cj = cbody.jx;    
    c_inertia = cbody.inertia;
    
    pos = x_wc + quatrotate( q_wc ...
                            , x_cj ...                                   
                            );
end