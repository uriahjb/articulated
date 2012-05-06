%{
  Function to solve for impulse to fulfil position constraint
    - this is painfully messy 
%}
function fj = f_pos_mod( j )

    global pbody cbody dt

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

    int_omega_p = omega_wp.*dt + dt.*(p_inertia\cross( j_xp, j )')';
    q_phat = q_hat( int_omega_p );
    
    % Don't forget this minus sign !!!!, there is something weird 
    % with these signs
    int_omega_c = omega_wc.*dt - dt.*(c_inertia\cross( j_xc, j)')';
    q_chat = q_hat( int_omega_c );
            
    fj  = x_wp + dt.*v_wp + dt.*j/m_p ...
        - x_wc - dt.*v_wc + dt.*j/m_c ...
        + quatrotate(q_phat, quatrotate( q_wp, x_pj + quatrotate( q_pj, xtarget ))) ...
        - quatrotate(q_chat, quatrotate( q_wc, x_cj ));   
end
    
    
    
    
    
