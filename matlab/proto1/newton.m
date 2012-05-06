%{
Newton Iteration as per interwebs 

We stop when we have epsilon times the initial error amount

f = @(x)((x-4).^2-4);
dfdx = @(x)(2.*(x-4));
%}
function x = newton(f, dfdx, x0, pbody, cbody, dt, epsilon)    
    err = Inf;
    x = x0;
    cnt = 0;
    errs = [];
    err_init = norm(f(x, pbody, cbody, dt));    
    
    if err_init == 0.0
        return
    end
    
    while true
       xPrev = x;     
       %{
       disp(['x: ' num2str(x)])
       disp(['xPrev: ' num2str(xPrev)])
       disp(['f: ' num2str(f(xPrev, pbody, cbody, dt))]);
       disp(['dfdx: ' num2str(dfdx(xPrev, pbody, cbody, dt))]);
       disp(['f./dfdx: ' num2str(f(xPrev, pbody, cbody, dt)./dfdx(xPrev, pbody, cbody, dt))]);
       %}       
       % stop criterion: (f(x) - 0) < tolerance
       jac = jacob( f, xPrev, pbody, cbody, dt );
       jac_sz = size(jac);
       if jac_sz(1) == jac_sz(2)            
            % I'm throwing in a quick hack
            x = xPrev - (jac\f(xPrev, pbody, cbody, dt)')';            
            x1 = xPrev - (jac'\f(xPrev, pbody, cbody, dt)')';
            err = f(x, pbody, cbody, dt);
            err1 = f(x1, pbody, cbody, dt);
            disp('err')            
            norm(err)
            disp('err1')
            norm(err1)
            if norm(err) < norm(err1) 
                x = x;
                err = err;
            else
                x = x1;
                err = err1;
            end

       else
            x = xPrev - (jac'\f(xPrev, pbody, cbody, dt)')';
       end
       %x = xPrev - f(xPrev, pbody, cbody, dt)./dfdx( xPrev, pbody, cbody,
       %dt );
       err = f(x, pbody, cbody, dt);
       %input('');
       
       if norm(err) < epsilon*err_init
           %disp('break?')
           norm(err)
           break
       end
       
       errs = [errs norm(err)];       
       if length(errs) > 2
           %disp('derr')
           derr = errs(end) - errs(end-1);
           if derr > 0 
               disp( ' derr is not decreasing, wtf' );
           end
       end
       cnt = cnt + 1;
       
       if norm(err) < 1e-6
           %norm(err)
           break
       end
       
       figure(2);
       plot( 1:cnt, errs );  
       %drawnow;
       % stop criterion: change of x < tolerance
       % err = x - xPrev;
    end      
    %norm(err)
end