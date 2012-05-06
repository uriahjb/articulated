%{
    Specify properties cube in the world, 
    I'm sticking with only these for now, time constraints are killing me
%}
classdef body < handle 
    properties
        h
        verts
        faces
        state
        dstate
        mass
        inertia  
        impulses
    end
    methods
        function self = body( state, dstate, mass, inertia, color )
            % Cube has default size of one
            xmin = -0.5;
            xmax = 0.5;
            ymin = xmin;
            ymax = xmax;
            zmin = xmin;
            zmax = xmax;
            patch_verts = [xmin ymin zmin; xmax ymin zmin; ...
                           xmax ymax zmin; xmin ymax zmin; ...
                           xmin ymin zmax; xmax ymin zmax; ...
                           xmax ymax zmax; xmin ymax zmax];
            patch_faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];    
            ph = patch;
            % clear vertices, we don't want to draw yet
            set(ph, 'vertices', [] );
            % Coloring all blue and transparent
            set(ph, 'FaceColor', color);
            set(ph, 'FaceAlpha', 0.7);

            self.h = ph;
            self.verts = patch_verts;
            self.faces = patch_faces;
            self.state = state;
            self.dstate = dstate;
            self.mass = mass;
            self.inertia = inertia;
            self.impulses = {};
        end
        
        function queue_impulse( self, impulse, loc )
            self.impulses = [self.impulses; impulse loc];
        end
        
        function integrate_velocity( self )
            i_sz = size(self.impulses);
            if self.mass == Inf
                self.impulses = [];
                return
            end
            for i = 1:i_sz(1)    
                impulse = self.impulses(i,1:3);
                loc = self.impulses(i,4:6);
                self.dstate(1:3) = self.dstate(1:3) + impulse./self.mass;
                self.dstate(4:6) = self.dstate(4:6) ...
                                 + (self.inertia\cross( loc, impulse )')';
            end
            self.impulses = [];
        end
        
        function integrate_position( self, dt )
            self.state(1:3) = self.state(1:3) + self.dstate(1:3).*dt;
            q_h = q_hat( self.dstate(4:6).*dt );    
            self.state(4:7) = quatmultiply( q_h, self.state(4:7) ); 
        end
        
        function draw( self )
            vts = quatrotate( self.state(4:7), self.verts );    
            vts = vts' + repmat( self.state( 1:3 )', 1, length(self.verts));
            set(self.h, 'vertices', vts', 'faces', self.faces ); 
        end
    end
end
        