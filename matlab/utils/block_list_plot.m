% Plot a list blocks on the given axis handle ax,
% p1, xmin ymin zmin
% p2, xmax ymax zmax
% c, color in r g b (0-255)

function ph = block_list_plot(ax, p1, p2, c)

v = [0 0 0; ...
    1 0 0; ...
    1 1 0; ...
    0 1 0; ...
    0 0 1; ...
    1 0 1; ...
    1 1 1; ...
    0 1 1];
f = [1 2 6 5; ...
    2 3 7 6; ...
    3 4 8 7; ...
    4 1 5 8; ...
    1 2 3 4; ...
    5 6 7 8];
c = c / 255;
d   = p2 - p1;
N   = size(p1,2);
vl  = zeros(8*N,3);
fl  = zeros(6*N,4);
fcl = zeros(6*N,3);
for k = 1:size(p1,2)
    vs = v;
    vs(:,1) = vs(:,1) * d(1,k);
    vs(:,2) = vs(:,2) * d(2,k);
    vs(:,3) = vs(:,3) * d(3,k);
    vs(:,1) = vs(:,1) + p1(1,k);
    vs(:,2) = vs(:,2) + p1(2,k);
    vs(:,3) = vs(:,3) + p1(3,k);
    vl((k-1)*8+1:k*8,:) = vs;
    fl((k-1)*6+1:k*6,:) = f + (k-1)*8;
    fcl((k-1)*6+1:k*6,:) = [c(:,k)';c(:,k)';c(:,k)';c(:,k)';c(:,k)';c(:,k)'];
end;
ph = patch('Vertices',vl, 'Faces',fl, 'FaceColor','flat', 'FaceVertexCData',fcl, 'Parent', ax, 'EdgeAlpha',0.2);%, 'SpecularColorReflectance',1, 'SpecularExponent',20, 'SpecularStrength', 1);
                 %'EdgeLighting','flat', 'FaceLighting', 'flat', 

end
