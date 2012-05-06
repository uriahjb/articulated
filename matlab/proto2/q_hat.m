%{
I think this is the formula to go from axis angle to quaternion, 
but I don't really remember ... will look up later
%}
function qdot = q_hat( w )
    if norm(w) > 0.0
        qdot = [cos(norm(w)/2) sin(norm(w)/2).*w/norm(w)]; 
    else              
        qdot = [1 0 0 0];
    end  
end