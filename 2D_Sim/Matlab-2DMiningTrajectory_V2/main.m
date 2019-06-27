
%% Setup

%clc
%clear all

%-----Import Parameters structure P-----
Parameters

%-----Plot black background and initial tunnel------
figure(1)
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

%------Create robot object--------
rob1 = Robot(P);
rob2 = Robot(P);


%***********************************
%------Trajectory Parameters--------
%***********************************
%Increase step length at each iteration. Also reduce number of iterations.
stepFactor = 2;

%reverseFactor = 0.7;
switchFactor = 1;

N = round(300/stepFactor);
skip = round(10/stepFactor);

vel_ = [0.30;0.02]*stepFactor;
om_ = 0.1*degToRad*stepFactor;
%***********************************
%-----------------------------------
%***********************************

%Bit Centroid coordinates
x = zeros(N,1);
y = zeros(N,1);

%-----Trajectory Simulation--------
tic
vel = vel_;
om = om_;
for j=1:N
    if j > N/switchFactor
        vel(2) = -vel_(2);
        om = -om_;
    end
    for i=2:length(rob1.Bodies)        
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
    rob1 = rob1.Move(vel,om);
end

%Plotting Options
axis('equal')
axis([15,P.map_x_max*0.8,100,P.map_y_max*0.95])
xlabel('x')
ylabel('y')
title(['Mining bot front body trajectory. vel = [',num2str(vel(1)),', ',num2str(vel(2)),'] in/step, om = ',num2str(om/degToRad),' deg/step']);


vel = vel_;
om = om_;
pg_animate = [];
for j=1:N
    if j > N/switchFactor
        vel(2) = -vel_(2);
        om = -om_;
    end
    if mod(j,skip) == 0 || j == 1 || j == N
        for i=1:length(rob2.Bodies)
            xverts = rob2.Bodies(i).movedVerts(:,1);
            yverts = rob2.Bodies(i).movedVerts(:,2);
            ps = polyshape(xverts,yverts);
            if j == 1
                pg_animate = [pg_animate,plot(ps)];
            else
                pg_animate(i).Shape = ps;
            end
            
            if i == 1
                pg_animate(i).FaceColor = 'blue';
                pg_animate(i).FaceAlpha = 0.6;
            else
                pg_animate(i).FaceColor = 'red';
                pg_animate(i).FaceAlpha = 1;
            end
        end
        drawnow
        pause(0.1)
    end
    
    rob2 = rob2.Move(vel,om);
end
toc

%Plot centroid path
plot(x,y,'b.');

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

