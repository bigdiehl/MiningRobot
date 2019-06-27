classdef Animation < handle
    %
    %    Create Ball on Beam (BOB) animation
    %
    %--------------------------------
    properties
        % drawing parameters
        box_handle
        spring_handle
        damper_handle
        L
        z_max
        H
        ne
        a
        r0
         
        
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = Animation(P)
            
            self.L = P.L; %Size of box.
            self.z_max = P.z_max; %Length of track both ways from z = 0
            self.H = P.H; %Height of track.
            self.ne = P.ne; %Next three are spring Variables. See drawSpring.m
            self.a = P.a;  
            self.r0 = P.r0;  
            
            
            figure(1), clf                       
            plot([-self.z_max,self.z_max],[0,0],'k-');     %plot main track
            hold on  
            plot([-self.z_max,-self.z_max],[0,self.H], 'k-');   %plot back-wall
            quiver(0,-0.5*self.H, self.z_max*.4, 0, 'k');  %Create z reference arrow
            plot([0,0],[-0.40*self.H,-.60*self.H], 'k-');  %Create z reference arrow
            text(-0.1*self.z_max, -0.5*self.H, 'z');       %Create z reference arrow
            self.drawBox(P.z0);
            self.drawSpring(P.z0);
            self.drawDamper(P.z0);
            axis equal
            axis([-1.2*self.z_max, 1.2*self.z_max, -1*self.H, 2*self.H]); %set axis sizes 
        end
        
        %---------------------------
        function self = drawBox(self, z)
            
            pts = [ z+0, z+self.L, z+self.L, z+0; 0, 0, self.L, self.L]; %Add z value to box
            %of size L with left edge centered at z = 0. Translates box
            %left and right along z-axis.
    
            X = pts(1,:);
            Y = pts(2,:);

            if isempty(self.box_handle)
                self.box_handle = fill(X,Y,'b');
            else
                set(self.box_handle,'XData',X,'YData',Y);
            end
            
        end
        %---------------------------
        function self = drawDamper(self, z)
            pts = [ -self.z_max, z ; self.L/4, self.L/4]; %Just a straight line for now.
    
            X = pts(1,:);
            Y = pts(2,:);

            if isempty(self.damper_handle)
                self.damper_handle = plot(X,Y,'g');
            else
                set(self.damper_handle,'XData',X,'YData',Y);
            end
        end
        %---------------------------
        
        function self = drawSpring(self,xb)
            % SPRING         Calculates the position of a 2D spring
            %    [XS YS] = SPRING(XA,YA,XB,YB,NE,A,R0) calculates the position of
            %    points XS, YS of a spring with ends in (XA,YA) and (XB,YB), number
            %    of coils equal to NE, natural length A, and natural radius R0. 
            %    Useful for mass-spring oscillation animations.
            % USAGE: in a first call in your code, call it with the full parameters.
            % Then, only you have to give it the coordinates of the ends.
            % EXAMPLE:
            % xa = 0; ya = 0; xb = 2; yb = 2; ne = 10; a = 1; ro = 0.1;
            % [xs,ys] = spring(xa,ya,xb,yb,ne,a,ro); plot(xs,ys,'LineWidth',2)
            %...
            % [xs,ys]=spring(xa,ya,xb,yb); plot(xs,ys,'LineWidth',2)
            %
            %   Made by:            Gustavo Morales   UC  08-17-09 gmorales@uc.edu.ve
            %
            %   Modified by: Spencer Diehl, September 2017
            %   Added modifications to work with plot handle methods.

            persistent Li_2 ei b
            
            % ne: number of coils - a = natural length - r0 = natural radius
            Li_2 =  (self.a/(4*self.ne))^2 + self.r0^2;                % (large of a quarter of coil)^2
            ei = 0:(2*self.ne+1);                            % vector of longitudinal positions
            j = 0:2*self.ne-1; b = [0 (-ones(1,2*self.ne)).^j 0]; % vector of transversal positions
            R = [xb 0.75*self.L] - [-self.z_max 0.75*self.L]; mod_R = norm(R); % relative position between "end_B" and "end_A"
            L_2 = (mod_R/(4*self.ne))^2; % (actual longitudinal extensin of a coil )^2
            
            if L_2 > Li_2
                error('Spring:TooEnlargement', ...
                'Initial conditions cause pulling the spring beyond its maximum large. \n Try reducing these conditions.')
            else
                r = sqrt(Li_2 - L_2);   %actual radius
            end
            c = r*b;    % vector of transversal positions
            u1 = R/mod_R; u2 = [-u1(2) u1(1)]; % unitary longitudinal and transversal vectors 
            xs = -self.z_max + u1(1)*(mod_R/(2*self.ne+1)).*ei + u2(1)*c; % horizontal coordinates
            ys = 0.75*self.L + u1(2)*(mod_R/(2*self.ne+1)).*ei + u2(2)*c; % vertical coordinates

            if isempty(self.spring_handle)
                self.spring_handle = plot(xs,ys, 'LineWidth' ,2);
            else
                set(self.spring_handle, 'XData',xs,'YData',ys);
            end
        end
    end
end