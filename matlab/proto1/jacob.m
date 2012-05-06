   function j = jacob(f, x, pbody, cbody, dt )                                % approximately calculate Jacobian matrix
   k = length(x);
   v = length(f(x, pbody, cbody, dt));
   j = zeros(k, v);
      
   for m = 1:k
       x2 = x;
       x2(m) =x(m)+0.001;
       f2 = f(x2, pbody, cbody, dt);
       f1 = f(x, pbody, cbody, dt);       
       j(m, :) = 1000*(f2-f1);        % partial derivatives in m-th row      
   end