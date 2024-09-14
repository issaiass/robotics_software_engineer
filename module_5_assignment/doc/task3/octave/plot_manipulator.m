% plot_manipulator(0.075, 0.5, 0.5, 0.005)

function plot_manipulator(L1, L2, L3, L4)
    % L1 = 0.075
    % L2 = 0.5
    % L3 = 0.5
    % L4 = 0.005

    % Sim steps
    steps = 50;

    figure;
    hold on;
    grid on;

    % Axes limits and labels
    axis([-3, 3, -3, 3, 0, 3]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Dynamic Moving Manipulator Animation');
    view(3);

    % plot handles
    link1_plot = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 3);  % Link 1
    link2_plot = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 3);  % Link 2
    link3_plot = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 3);  % Link 3
    link4_plot = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 3);  % Link 4

    % ee information (text)
    end_effector_text = text(0, 0, 0, '', 'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');

    % Arrow handles
    [joint1_arrows, joint2_arrows, joint3_arrows, end_effector_arrows] = init_axes_arrows();

    % initial angles of joints (prismatic just have position in z)
    theta_init1 = 0;
    theta_init2 = 0;
    theta_init3 = 0;

    % Loop for dynamic animation for 10 loops
    for n = linspace(0, 10)
        for t = linspace(0, 2 * pi, steps)
            % Define dynamic joint angles (moving with time)
            theta1 = theta_init1 + sin(t);   % base joint oscilation
            theta2 = theta_init2 + sin(2*t); % finger joint oscilation
            theta3 = theta_init3;            % ee joint angle
            #theta2 = theta_init2 + cos(t);   % Joint 2 angle (oscillating)
            #theta3 = theta_init3 + sin(t/2); % Joint 3 angle (slower oscillation)

            % FK Matrices
            T01 =  trotz(theta1) * transl([0, 0, L1]);  % From base to first link (base)
            T12 = troty(theta2) * transl([0, 0, L2]);  % From first link to second (finger)
            T23 = transl([0, 0, L3 - 0.2]);  % From second link to third (extension)
            T34 = trotz(theta3) * transl([0, 0, L4]);  % From second link to third (ee)

            % Calculate the end-effector position
            T02 = T01 * T12;  % From base to finger link
            T03 = T02 * T23;  % From base to prismatic
            T04 = T03 * T34;  % From base to ee link

            % Extract the positions of each joint
            origin = [0; 0; 0];          % Base position
            joint1 = T01(1:3, 4);        % Position of the first joint
            joint2 = T02(1:3, 4);        % Position of the second joint
            joint3 = T03(1:3, 4);        % Position of the third joint
            end_effector = T04(1:3, 4);  % Position of the end-effector

            % Calculate the end-effector orientation (theta around the Z-axis)
            theta_ee = atan2(T04(2,1), T04(1,1));  % Extract the rotation angle around Z-axis

            % Static memory to plot
            set(link1_plot, 'XData', [origin(1), joint1(1)], 'YData', [origin(2), joint1(2)], 'ZData', [origin(3), joint1(3)]);
            set(link2_plot, 'XData', [joint1(1), joint2(1)], 'YData', [joint1(2), joint2(2)], 'ZData', [joint1(3), joint2(3)]);
            set(link3_plot, 'XData', [joint2(1), joint3(1)], 'YData', [joint2(2), joint3(2)], 'ZData', [joint2(3), joint3(3)]);
            set(link4_plot, 'XData', [joint3(1), end_effector(1)], 'YData', [joint3(2), end_effector(2)], 'ZData', [joint3(3), end_effector(3)]);

            % Update update arrows
            update_axes_arrows(joint1_arrows, T01, joint1);
            update_axes_arrows(joint2_arrows, T02, joint2);
            update_axes_arrows(joint3_arrows, T03, joint3);
            update_axes_arrows(end_effector_arrows, T04, end_effector);

            % Update the text showing the end-effector position and orientation
            % Slight offset for readability
            set(end_effector_text, 'Position', [end_effector(1)+1, end_effector(2)+1, end_effector(3)+1], ...
                'String', sprintf('X: %.2f\nY: %.2f\nZ: %.2f\nTheta: %.2f', ...
                end_effector(1), end_effector(2), end_effector(3), theta_ee));
            % enable to view the 3d space position
            %printf("{%.2f, %.2f, %.2f},\n", end_effector(1), end_effector(2), end_effector(3))

            % enable to view joint positions
            %printf("{%.6f, %.6f, %.6f, %.6f},\n", joint1, joint2, joint3, end_effector)
            % Pause for animation
            %pause(0.05);

            % Update plot
            drawnow;
        end
    end
    hold off;
end

% Functions of rotation and translation (due to i do not have RoboticsToolbox)
function T = trotz(theta)
  T = [cos(theta), -sin(theta), 0, 0;
          sin(theta),  cos(theta), 0, 0;
          0,           0,          1, 0;
          0,           0,          0, 1];
end

function T = troty(theta)
    T = [cos(theta), 0, sin(theta), 0;
          0, 1, 0, 0;
          -sin(theta), 0, cos(theta), 0;
          0, 0, 0, 1];
end

function T = transl(t)
    T = [1, 0, 0, t(1);
         0, 1, 0, t(2);
         0, 0, 1, t(3);
         0, 0, 0, 1];
end

% Initialize the axes arrows for each joint (X, Y, Z axes)
function [joint1_arrows, joint2_arrows, joint3_arrows, end_effector_arrows] = init_axes_arrows()
    % X (red), Y (green), Z (blue)
    joint1_arrows = [
        quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 1.5);
    ];

    joint2_arrows = [
        quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 1.5);
    ];

    joint3_arrows = [
        quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 1.5);
    ];

    end_effector_arrows = [
        quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 1.5);
        quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 1.5);
    ];
end

% Update the axes arrows for a given joint
function update_axes_arrows(arrow_handles, T, origin)
    % Extract rotation matrix (upper left 3x3) from transformation matrix
    R = T(1:3, 1:3);
    arrow_length = 0.5;

    % X-axis (Red)
    set(arrow_handles(1), 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
        'UData', R(1,1)*arrow_length, 'VData', R(2,1)*arrow_length, 'WData', R(3,1)*arrow_length);

    % Y-axis (Green)
    set(arrow_handles(2), 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
        'UData', R(1,2)*arrow_length, 'VData', R(2,2)*arrow_length, 'WData', R(3,2)*arrow_length);

    % Z-axis (Blue)
    set(arrow_handles(3), 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
        'UData', R(1,3)*arrow_length, 'VData', R(2,3)*arrow_length, 'WData', R(3,3)*arrow_length);
end
