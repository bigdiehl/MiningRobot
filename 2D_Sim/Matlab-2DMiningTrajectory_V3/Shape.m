%Class to compute and contain the vertices and points along the perimeter of a 2D 
%rectangle in the body frame with a given offset. Contains the original
%points and the points that have been rotated and translated by other
%functions.
classdef Shape
    properties
        Verts
        movedVerts
        width 
        length
        offset
        shapeType %1=rectangle, 2=half ellipse 
    end
    %----------------------------
    methods
        function self = Shape(length, width, x_offset, y_offset, shapeType)
            self.length = length;
            self.width = width;
            self.offset = [x_offset, y_offset];
            self.shapeType = shapeType;
            
            if shapeType == 1
                %Original vertices in the base frame (Always keep a copy)
                self.Verts = self.boxPoints(length, width) + self.offset;
            elseif shapeType == 2
                %Original vertices in the base frame (Always keep a copy)
                self.Verts = self.halfellipsePoints(length, width) + self.offset;
            end
        end
        %----------------------------
        function verts = boxPoints(~, length, width)
            verts = [-length/2,-width/2;
                     -length/2, width/2;
                      length/2, width/2; 
                      length/2,-width/2];
        end
        %----------------------------
        function verts = halfellipsePoints(~, len, width)
            theta = linspace(-pi/2, pi/2, 10);
        
            ry = width/2;
            rx = len/2;
            
            x = zeros(length(theta),1);
            y = zeros(length(theta),1);
            
            for i=1:length(theta)
                x(i) = rx*cos(theta(i));
                y(i) = ry*sin(theta(i));
            end
            verts = [x,y];
        end
        %----------------------------
    end
end