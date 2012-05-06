%{
    Make a cube
%}
function body = mk_cube( state, dstate, mass, inertia, color ) 
    % Cube has default size of one
    xmin = -0.5;
    xmax = 0.5;
    ymin = xmin;
    ymax = xmax;
    zmin = xmin;
    zmax = xmax;
    verts = [xmin ymin zmin; xmax ymin zmin; ...
             xmax ymax zmin; xmin ymax zmin; ...
             xmin ymin zmax; xmax ymin zmax; ...
             xmax ymax zmax; xmin ymax zmax];
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];    
    h = patch;
    % clear vertices, we don't want to draw yet
    set(h, 'vertices', [] );
    % Coloring all blue and transparent
    set(h, 'FaceColor', color);
    set(h, 'FaceAlpha', 0.7);
    
    body.h = h;
    body.verts = verts;
    body.faces = faces;
    body.state = state;
    body.dstate = dstate;
    body.mass = mass;
    body.inertia = inertia;
end