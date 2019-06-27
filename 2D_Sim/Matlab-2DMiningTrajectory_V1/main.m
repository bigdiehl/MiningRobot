

%% Testing
if(0)
    clc; clear all;

    figure(1)
    clf

    ps = polyshape([0 0 20 20],[20 0 0 20]);
    pg = plot(ps);
    pg.FaceColor = 'black';
    pg.FaceAlpha = 1;

    hold on
    for i=1:10
        ps = polyshape([0 0 1 1]+i,[1 0 0 1]+i);
        pg = plot(ps);
        pg.FaceColor = 'white';
        pg.FaceAlpha = 1;
    end

    axis('equal')
end


%Define polygons for the mining components

%% Setup
clc
clear all
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

degToRad = pi/180;

stepFactor = 2;

%reverseFactor = 0.7;
switchFactor = 1;

N = round(300/stepFactor);
skip = round(100/stepFactor);

vel_ = [0.30;0.025]*stepFactor;
om_ = 0.082*degToRad*stepFactor;

x = zeros(N,1);
y = zeros(N,1);

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

vel = vel_;
om = om_;
for j=1:N
    if j > N/switchFactor
        vel(2) = -vel_(2);
        om = -om_;
    end
    for i=1:length(rob2.Bodies)
        if mod(j,skip) == 0 || j == 1 || j == N
            xverts = rob2.Bodies(i).movedVerts(:,1);
            yverts = rob2.Bodies(i).movedVerts(:,2);
            ps = polyshape(xverts,yverts);
            pg = plot(ps);
            if i == 1
                pg.FaceColor = 'blue';
                pg.FaceAlpha = 0.6;
            else
                pg.FaceColor = 'red';
                pg.FaceAlpha = 1;
            end
        end
    end
    rob2 = rob2.Move(vel,om);
end
toc

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


axis('equal')
axis([15,P.map_x_max*0.8,100,P.map_y_max*0.95])
xlabel('x')
ylabel('y')
title(['Mining bot front body trajectory. vel = [',num2str(vel(1)),', ',num2str(vel(2)),'] in/step, om = ',num2str(om/degToRad),' deg/step']);
