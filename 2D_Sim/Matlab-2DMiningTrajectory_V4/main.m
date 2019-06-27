
%% Setup

clc
clear all

%-----Import Parameters structure P-----
Parameters

%-----Plot black background and initial tunnel------
figure(2)
clf
ps = polyshape([0 0 P.map_x_max P.map_x_max],[P.map_y_max 0 0 P.map_y_max]);
pg = plot(ps);
pg.FaceColor = 'black';
pg.FaceAlpha = 1;
hold on

ps = polyshape([0 0 P.init_tunnel_length P.init_tunnel_length]+P.x0,[P.init_tunnel_width 0 0 P.init_tunnel_width]+P.y0-P.init_tunnel_width/2);
pg = plot(ps);
pg.FaceColor = 'white';
pg.EdgeColor = 'white';
pg.FaceAlpha = 1;

%Plotting Options
axis('equal')
axis([15,P.map_x_max*0.8,150,P.map_y_max*0.95])
xlabel('x')
ylabel('y')

%Axis object
ax = gca;

%------Create robot object--------
rob1 = Robot(P);
rob2 = Robot(P);

%***********************************
%------Trajectory Parameters--------
%***********************************
%Increase step length at each iteration. Also reduce number of iterations.
numCycles = 20;

skip = 2;
stepFactor = 1;

%vel = [(P.d_max/10)*stepFactor;(P.d_max/140)*stepFactor*1.6];
vel = [(P.d_max/10)*stepFactor;0];
om  = 0.082*degToRad*stepFactor*0.5;

pause_time = 0.00001;
N = round((P.d_max/vel(1))*numCycles);
%***********************************
%-----------------------------------
%***********************************

%Bit Centroid coordinates
x = zeros(N,1);
y = zeros(N,1);

j = 0;
pg_animate = [];
%-----Trajectory Simulation--------
for k=1:numCycles
    
    %Step 1 - Extend and pivot
    while rob1.d < P.d_max
        j = j+1;
        
        %Update hole
        for i=3:length(rob1.Bodies)       
            xverts = rob1.Bodies(i).movedVerts(:,1);
            yverts = rob1.Bodies(i).movedVerts(:,2);
            ps = polyshape(xverts,yverts);
            pg = plot(ps);
            pg.FaceColor = 'white';
            pg.EdgeColor = 'white';
            pg.FaceAlpha = 1;
            if i==3
                [x(j),y(j)] = centroid(ps);
            end
        end
        
        %Update robot animation
        if mod(j,skip) == 0 || j == 1 || j == N
            for i=1:length(rob2.Bodies)
                xverts = rob2.Bodies(i).movedVerts(:,1);
                yverts = rob2.Bodies(i).movedVerts(:,2);
                
                if j == 1
                    ps = fill3(xverts,yverts,ones(length(xverts),1)*0.01,ones(length(xverts),1),'FaceColor','red','FaceAlpha',0.4);
                    
                    if i == 1
                        ps.FaceColor = 'blue';
                        ps.FaceAlpha = 0.6;
                    elseif i == 2
                        ps.FaceColor = 'blue';
                        ps.FaceAlpha = 0.6;
                    else
                        ps.FaceColor = 'red';
                        ps.FaceAlpha = 1;
                    end
                    pg_animate = [pg_animate,ps];
                else
                    pg_animate(i).Vertices = [xverts,yverts,ones(length(xverts),1)*0.01];
                end
                
            end
            drawnow
            pause(pause_time)
        end
                
        rob1.extend(vel(1));
        rob2.extend(vel(1));
        
        rob1.Move([0;vel(2)],0)
        rob2.Move([0;vel(2)],0)
        
        rob1.pivot(om);
        rob2.pivot(om);
    end
    
    %Step 2 - Retract 
    while rob1.d > 0
        j = j+1;
        
       %Update robot animation
        if mod(j,skip) == 0 || j == 1 || j == N
            for i=1:length(rob2.Bodies)
                xverts = rob2.Bodies(i).movedVerts(:,1);
                yverts = rob2.Bodies(i).movedVerts(:,2);
                
                pg_animate(i).Vertices = [xverts,yverts,ones(length(xverts),1)*0.01];
            end
            drawnow
            pause(pause_time)
        end
        rob1.retract(vel(1));
        rob2.retract(vel(1));
    end
end

%Plot centroid path
plot(x,y,'b.');
title(['Mining bot front body trajectory. vel = [',num2str(vel(1)),', ',num2str(vel(2)),'] in/step, om = ',num2str(om/degToRad),' deg/step']);


%-----Plot target curvature-----
theta = linspace(pi/2,pi/4,100);
r = P.turning_radius;
x = zeros(length(theta),1);
y = zeros(length(theta),1);
for i=1:length(theta)
    x(i) = r*cos(theta(i));
    y(i) = r*sin(theta(i));
end
x = x+P.x0+P.front_length;
y = y - (P.turning_radius-P.y0+P.cutter_width/2);
plot(x,y,'r--')

