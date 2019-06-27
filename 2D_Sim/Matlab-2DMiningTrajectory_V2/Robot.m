               
% Class to contain the movement variables and collection of Rectangle objects that make up the
% Front body (bit and pipe).
% 
% : y_body
% :
% |-----------|||\
% |   Front   |||||......... x_body
% |-----------|||/
classdef Robot
    properties
        theta
        pB
        
        FrontBody
        CutterBody
        Cutter
        
        Bodies
        ContactCheckBodies
        CutterBodies
    end
    %----------------------------
    methods
        function self = Robot(P)
        self.theta = 0;         %Rotation of Body frame from Origin frame
        self.pB = [P.x0; P.y0]; %Vector from base frame origin to body frame origin

        self.FrontBody  = Shape(P.front_length, P.front_width, P.front_length/2, 0, 1);
        self.CutterBody = Shape(P.cutter_length, P.cutter_width, P.front_length+P.cutter_length/2, 0, 1);
        self.Cutter = Shape(P.bit_length, P.cutter_width, P.front_length+P.cutter_length, 0, 2);

        %Set of bodies that must check for contact
        self.Bodies = [self.FrontBody, self.Cutter, self.CutterBody];
        self.ContactCheckBodies = [self.FrontBody];
        self.CutterBodies = [self.Cutter, self.CutterBody];
        
        %Initialize moved vertices
        self = self.Move([0;0],0);
        end
        %----------------------------
        %Updates the movedVertices variable which contains the robot body vertices in the origin frame.'''
        function self = Move(self, vel, om)
            self.theta = self.theta + om;
            R_b_o = self.rotZ(self.theta);
            self.pB = self.pB + (R_b_o*vel);

            %Update proposed body vertices
            for i=1:length(self.Bodies)
                self.Bodies(i).movedVerts = self.pB' + (R_b_o*self.Bodies(i).Verts')';
            end
        end
        %----------------------------
        function self = extend(self,vel)
            
        end
        %----------------------------
        %Returns a list of the body vertices
        function verts = getVerts(self)
            verts = [];
            for i=1:length(self.Bodies)
                verts = [verts,body.movedVertices];
            end
        end
        %----------------------------
        %Assumes points is a Nx2 np array and distance is a 2 element np array
        function points = translate(self, points, distance)
            points =  points + distance;
        end
        %----------------------------
        %Right handed in passive sense z axis rotation
        %(CCW is positive angle). Assumes points is a Nx2 np array
        function points = rotate(self, points, angle)
            rotZ = self.rotZ(angle);
            points = (rotZ*points')';
        end
        %----------------------------
        %Returns just the z rotation matrix. Left handed in passive sense.
        %(i.e. Rotates BACK to previous frame from origin.)
        function rot = rotZ(self, angle)
            rot = [ cos(angle), sin(angle);
                   -sin(angle), cos(angle)];
        end
        %----------------------------
    end
end