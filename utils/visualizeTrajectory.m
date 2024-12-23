function [] = visualizeTrajectory(robot, joint_traj, joint_dot_traj,...
    joint_ddot_traj, time_trajectory, start_figure_ind, S0, M0, animation)
% Visualize trajectory of EE fram and plot joint position/velocity/acceleration

theta_start = joint_traj(:,1);
theta_end = joint_traj(:,end);

%---------------- Visualize EE trajectory 
figure(start_figure_ind); clf;

% Start and End pose
show(robot,theta_start,'Visuals','on','Frames','off','Collisions','off');
hold on
show(robot,theta_end,'Visuals','on','Frames','off','Collisions','off');

% Visualize configuration
T_sb = FK_SpaceForm(S0, M0, theta_start);
triad('Matrix',T_sb,'Scale',0.3,'LineWidth',2,'linestyle','-');
T_sb = FK_SpaceForm(S0, M0, theta_end);
triad('Matrix',T_sb,'Scale',0.3,'LineWidth',2,'linestyle','-');

% Loop to take p(t)
[numJoint, N] = size(joint_traj); % N: Number of samples
p = zeros(3,N);
for i=1:N
    % FK to find Tsb
    T_sb = FK_SpaceForm(S0, M0, joint_traj(:,i));
    % Visualize EE frame
    if mod(i,10) == 0
        triad('Matrix',T_sb,'Scale',0.1,'LineWidth',1,'linestyle','-');
    end
    % Append position 
    p(:,i) = T_sb(1:3,4);
end
x = p(1,:);
y = p(2,:);
z = p(3,:);

% Plot p(t)
plot3(x,y,z);
hold off

% Title
title("End-Effector Frame Trajectory");
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")


%------------------ Plot p(t)
figure(start_figure_ind+1); clf;
subplot(2,2,1);
plot(time_trajectory,x,'r');
hold on
plot(time_trajectory,y,'g');
plot(time_trajectory,z,'b');
hold off
legend(["x","y","z"]);
title("p(t)");
ylabel("Distance (m)")
xlabel("time (s)")

%------------------- theta, theta_dot, theta_ddot
subplot(2,2,2);
plot(time_trajectory', joint_traj);
title("$\mathbf{\theta}(t)$", 'Interpreter', 'latex');
ylabel("\theta (rad)")
xlabel("time (t)")

subplot(2,2,3);
plot(time_trajectory',joint_dot_traj);
hold on 
yline(2.6180,'r--','LineWidth',2);
yline(-2.6180,'b--','LineWidth',2);
hold off
title('$\dot{\mathbf{\theta}}(t)$', 'Interpreter', 'latex');
ylabel("$\dot{\theta} (rad/s)$", 'Interpreter', 'latex')
xlabel("time (t)")

subplot(2,2,4);
plot(time_trajectory',joint_ddot_traj);
hold on 
yline(12.2173,'r--','LineWidth',2);
yline(-12.2173,'b--','LineWidth',2);
hold off
title('$\ddot{\mathbf{\theta}}(t)$', 'Interpreter', 'latex');
ylabel("$\ddot{\theta} (rad/s^2)$", 'Interpreter', 'latex')
xlabel("time (t)")


%---------------- Visualize robot motion
figure(start_figure_ind+2); clf;
% Start and end robot config
show(robot,theta_start,'Visuals','on','Frames','off','Collisions','off');
hold on
show(robot,theta_end,'Visuals','on','Frames','off','Collisions','off');
plot3(x,y,z);
hold off

if animation
    
    for i=1:N 
        theta_d = joint_traj(:,i);
        % Current robot config
        show(robot,theta_d,'PreservePlot',false,'Visuals','off','Frames','off','Collisions','on');
        hold on
        % Start and end robot config
        show(robot,theta_start,'Visuals','on','Frames','off','Collisions','off');
        show(robot,theta_end,'Visuals','on','Frames','off','Collisions','off');
        plot3(x,y,z);
        
        drawnow
    end
    hold off
end
