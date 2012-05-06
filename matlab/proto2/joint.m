classdef joint < handle 
    properties
        pbody 
        cbody        
        xtarget        
        qtarget
        utype
        wtype
        px
        cx
        pq
        cq
    end
    methods 
        function self = joint( pbody, cbody, utype, wtype, px, cx, pq, cq )
            self.pbody = pbody;
            self.cbody = cbody;
            self.utype = utype;
            self.wtype = wtype;
            self.px = px;
            self.cx = cx;
            self.pq = pq;
            self.cq = cq;
            self.xtarget = [0 0 0];
            self.qtarget = [1 0 0 0];
        end
        function set_target( self, xtarget, qtarget )
            self.xtarget = xtarget;
            self.qtarget = qtarget;
        end
        function evolve( self, dt )
            pbody = self.pbody;
            cbody = self.cbody;
            x_wp = pbody.state(1:3);
            q_wp = pbody.state(4:7);
            v_wp = pbody.dstate(1:3);
            omega_wp = pbody.dstate(4:6);            
            x_pj = self.px;
            q_pj = self.pq;

            x_wc = cbody.state(1:3);
            q_wc = cbody.state(4:7);
            v_wc = cbody.dstate(1:3);
            omega_wc = cbody.dstate(4:6);            
            x_cj = self.cx;
            q_cj = self.cq;            
            
            q_hp = q_hat( omega_wp.*dt );    
            q_wp1 = quatmultiply( q_hp, q_wp );

            q_hc = q_hat( omega_wc.*dt );    
            q_wc1 = quatmultiply( q_hc, q_wc );

            x_wp1 = x_wp + v_wp.*dt;
            x_wc1 = x_wc + v_wc.*dt;

            xnew = quatrotate( quatinv(q_pj), ( quatrotate( quatinv(q_wp1), ( x_wc1 + quatrotate( q_wc1, x_cj ) - x_wp1 )) - x_pj ) );   
            qnew = quatmultiply( quatinv(q_pj), quatmultiply( quatinv(q_wp1), quatmultiply( q_wc1, q_cj )));       
            
            self.apply_constraints( xnew, qnew )
        end
        
        function apply_constraints( self, xnew, qnew )
            self.xtarget = xnew;
            self.qtarget = qnew;
        end
        
        function stabilize_velocity( self )
            pbody = self.pbody;
            cbody = self.cbody;
            utype = self.utype;
            wtype = self.wtype;
            
            x_wp = pbody.state(1:3);
            q_wp = pbody.state(4:7);
            v_wp = pbody.dstate(1:3);
            omega_wp = pbody.dstate(4:6);
            m_p = pbody.mass;    
            x_pj = pbody.jx;
            q_pj = pbody.jq;            
            p_inertia = pbody.inertia;

            x_wc = cbody.state(1:3);
            q_wc = cbody.state(4:7);
            v_wc = cbody.dstate(1:3);
            omega_wc = cbody.dstate(4:6);
            m_c = cbody.mass;    
            x_cj = cbody.jx;    
            c_inertia = cbody.inertia;
            
            xtarget = self.xtarget;

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
            apply_impulse( j );
            apply_torque( jt );
        end
        
        function apply_impulse( self, j )
            pbody = self.pbody;
            cbody = self.cbody;
            jposp = jpos_p( pbody );
            jposc = jpos_c( cbody );
            pbody.dstate(1:3) = pbody.dstate(1:3) + j./pbody.mass;
            pbody.dstate(4:6) = pbody.dstate(4:6) ...
                             + (pbody.inertia\cross( jposp, j )')';
            cbody.dstate(1:3) = cbody.dstate(1:3) - j./cbody.mass;
            cbody.dstate(4:6) = cbody.dstate(4:6) ...
                             - (cbody.inertia\cross( jposc, j )')';    
        end
        
        function apply_torque( self, jt )            
            pbody = self.pbody;
            cbody = self.cbody;
            jposp = jpos_p( pbody );
            jposc = jpos_c( cbody );
            pbody.dstate(1:3) = pbody.dstate(1:3);
            pbody.dstate(4:6) = pbody.dstate(4:6) ...
                              + (pbody.inertia\jt')';
            cbody.dstate(1:3) = cbody.dstate(1:3);
            cbody.dstate(4:6) = cbody.dstate(4:6) ...
                              - (cbody.inertia\jt')';                  
        end                
    end
end
