%{
    Draw body:
        Just apply current state to vertices and drawnow
%}
function draw_body( body )
    % Apply transform to verts and update the object
    %verts = QuatToRot( body.state(4:7) )*body.verts';
    
    % Doing rotation with quaternions, whoo!
    %{
    verts = quatmultiply( quatmultiply( body.state(4:7) ...
                                      , [zeros(length(body.verts),1) body.verts] ) ...
                        , quatinv( body.state(4:7) ) );
    %}
    % Rotate with quaternions ... haha this already exists    
    verts = quatrotate( body.state(4:7), body.verts );    
    verts = verts' + repmat( body.state( 1:3 )', 1, length(body.verts));
    set(body.h, 'vertices', verts', 'faces', body.faces );    
end