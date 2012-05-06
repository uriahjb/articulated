function qdot = q_hat2( w, q )
    A = [q(1) q(4) -q(3) q(2); ...
         q(2) q(3)  q(4) -q(1); ...
         q(3) -q(2) q(1) q(4); ...
         q(4) -q(1) -q(2) -q(3)];
    W = [0 w]';
    
    qdot = 0.5.*A*W;
end
        