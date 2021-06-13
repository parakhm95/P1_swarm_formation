%%
pos = randi(30,6,2); % random positions for starting of the drones
% initial formation of a hexagon centered on origin
final = [[(3^0.5)/2, 0.5];
    [0,1];
    [-(3^0.5)/2, 0.5];
    [-(3^0.5)/2, -0.5];
    [0,-1];
    [(3^0.5)/2, -0.5]];
% matrix where we store our positions during the loop for plotting
pos1 = [];
pos2 = [];
pos3 = [];
pos4 = [];
pos5 = [];
pos6 = [];
% matrix where we store the forces experienced by each robot for plotting
force_collect = [];
% temporary force matrix to be used for every robot/aircraft/drone
force_temp = zeros(1,2);
% activation radius for our force of repulsion
rs = 3.0;
% time chunk that we are simulating to be going through
dt = 0.03; % 33 hz. 
kp = 0.85; % formation_control_gain
kc = 1.5; % formation_movement_gain
N = 6; % number of robots
alpha = 0.5; % replusion_gain 
beta = 0.3; % repulsion_ramp_gain
plot(final(:,1),final(:,2),'--r*'); % plotting our final destination/formation goal
hold on; % don't erase the things on the plot. Keep them there. 
for t=0:dt:10 % 0 seconds to 10 seconds 0,0.03,0.06....10
    dv=zeros(6,2); % velocity matrix for every instance
    force=zeros(6,2); % force vector matrix for every instance
    for i=1:1:N
        dv_temp = [0.0,0.0]; % temporary velocity variable
        for j=1:1:N % each robot
            dv_temp = dv_temp + pos(j,:)-pos(i,:)-(final(j,:)-final(i,:)); % velocity for formation control
            % force equation
            if norm(pos(j,:)-pos(i,:)) > 0 && norm(pos(j,:) - pos(i,:)) < rs % activate when below rs distance
                force_repel = alpha*(exp(-beta*norm(pos(j,:)-pos(i,:))) - exp(-beta*rs));  % scalar force     
                force_temp = force_temp + (pos(j,:)-pos(i,:))/norm(pos(j,:)-pos(i,:))*force_repel; % vector force
            else
                force_temp = [0,0];
            end
        end
        dv(i,1) = dv_temp(1); % velocity x 
        dv(i,2) = dv_temp(2); % velocity y
        force(i,:) = force_temp;
        force_collect = [force_collect; force_temp];
    end
    % calculation of centroid
    centroid = zeros(1,2);
    for i=1:1:6
        centroid = centroid + pos(i,:);
    end
    centroid = centroid/6;
    for i=1:1:6
        pos(i,1) = pos(i,1) + (kp*dv(i,1) - centroid(1,1)*kc - force(i,1))*dt;
        pos(i,2) = pos(i,2) + (kp*dv(i,2) - centroid(1,2)*kc - force(i,2))*dt;
    end
    pos1 = [pos1;pos(1,:)];
    pos2 = [pos2;pos(2,:)];
    pos3 = [pos3;pos(3,:)];
    pos4 = [pos4;pos(4,:)];
    pos5 = [pos5;pos(5,:)];
    pos6 = [pos6;pos(6,:)];
    plot(pos(1,1),pos(1,2),'r.');
    hold on;
    plot(pos(2,1),pos(2,2),'b.');
    hold on;
    plot(pos(3,1),pos(3,2),'g.');
    hold on;
    plot(pos(4,1),pos(4,2),'k.');
    hold on;
    plot(pos(5,1),pos(5,2),'m.');
    hold on;
    plot(pos(6,1),pos(6,2),'c.');
    hold on;
    pause(dt);
end
%%
% plot(final(:,1),final(:,2),'--r*');