function kinematics_gui2()
    % Create the main figure
    fig = figure('Name', 'Kinematics GUI', 'Position', [100, 100, 400, 300]);

    % Create dropdown menu
    dropdown = uicontrol('Style', 'popupmenu', 'String', {'Forward Kinematics', 'Inverse Kinematics', 'Workspace'}, ...
                         'Position', [50, 230, 300, 40], 'Callback', @dropdown_callback);

    % Dropdown callback function
    function dropdown_callback(source, ~)
        option = source.Value;
        switch option
            case 1 % Forward Kinematics
                forward_window();
            case 2 % Inverse Kinematics
                inverse_window();
            case 3 % Workspace
                workspace_window(); % Add Workspace option
        end
    end

    % Function to create window for forward kinematics input
    function forward_window()
        forward_fig = figure('Name', 'Forward Kinematics', 'Position', [500, 300, 600, 400]);
        
        % Create input text fields for arrays
        input_labels = {'Theta', 'Alpha', 'D', 'A'};
        input_texts = cell(1, 4); % Define input_texts as a cell array
        start_y = 200; % Starting position for y-axis
        spacing = 30; % Spacing between controls
        
        for i = 1:length(input_labels)
            uicontrol('Style', 'text', 'String', input_labels{i}, 'Position', [50, start_y - i * spacing, 80, 20], 'Parent', forward_fig);
            input_texts{i} = uicontrol('Style', 'edit', 'Position', [150, start_y - i * spacing, 150, 20], 'Parent', forward_fig);
        end

        % Create output text box to display results
        result_text = uicontrol('Style', 'edit', 'Position', [50, 200, 300, 100], 'Parent', forward_fig, ...
            'HorizontalAlignment', 'left', 'Max', 2, 'Enable', 'inactive');

        % Create button to calculate forward kinematics
        uicontrol('Style', 'pushbutton', 'String', 'Calculate', 'Position', [150, 300, 100, 40], ...
            'Callback', {@calculate_forward, input_texts, result_text}, 'Parent', forward_fig);
    end

    % Function to create window for inverse kinematics input
    function inverse_window()
        inverse_fig = figure('Name', 'Inverse Kinematics', 'Position', [500, 300, 600, 400]);
        
        % Create input text fields for arrays
        input_labels = {'Alpha', 'inital Theta', 'A', 'D', 'EE Position'};
        input_texts = cell(1, 6); % Define input_texts with 6 elements
        start_y = 200; % Starting position for y-axis
        spacing = 30; 
        
        for i = 1:length(input_labels)
            uicontrol('Style', 'text', 'String', input_labels{i}, 'Position', [50, start_y - i * spacing, 80, 20], 'Parent', inverse_fig);
            input_texts{i} = uicontrol('Style', 'edit', 'Position', [150, start_y - i * spacing, 150, 20], 'Parent', inverse_fig);
        end

        % Create output text box to display results
        result_text = uicontrol('Style', 'edit', 'Position', [50, 200, 300, 100], 'Parent', inverse_fig, ...
            'HorizontalAlignment', 'left', 'Max', 2, 'Enable', 'inactive');

        % Create button to calculate inverse kinematics
        uicontrol('Style', 'pushbutton', 'String', 'Calculate', 'Position', [150, 300, 100, 40], ...
            'Callback', {@calculate_inverse, input_texts, result_text}, 'Parent', inverse_fig);
    end

    % Function to create window for workspace input
    function workspace_window()
        workspace_fig = figure('Name', 'Workspace', 'Position', [500, 300, 900, 400]);
        
        % Create input text fields for arrays
        input_labels = {'Alpha', 'A', 'D', 'Theta Max'};
        input_texts = cell(1, 4);
        start_y = 300; % Starting position for y-axis
        spacing = 30; 
        
        for i = 1:length(input_labels)
            uicontrol('Style', 'text', 'String', input_labels{i}, 'Position', [50, start_y - i * spacing, 80, 20], 'Parent', workspace_fig);
            input_texts{i} = uicontrol('Style', 'edit', 'Position', [150, start_y - i * spacing, 150, 20], 'Parent', workspace_fig);
        end

        % Create button to calculate workspace
        uicontrol('Style', 'pushbutton', 'String', 'Calculate', 'Position', [150, 30, 100, 40], ...
            'Callback', {@calculate_workspace, input_texts}, 'Parent', workspace_fig);
        
        % Create axes for plot
        ax = axes('Parent', workspace_fig, 'Position', [0.4, 0.15, 0.5, 0.7]);

        % Define initial plot data (empty)
        plot(ax, [], []);
        
        % Set plot properties
        title(ax, 'Workspace Plot');
        xlabel(ax, 'X');
        ylabel(ax, 'Y');
        zlabel(ax, 'Z');
        grid(ax, 'on');
    end

    % Function to calculate forward kinematics
    function calculate_forward(~, ~, input_texts, result_text)
        % Retrieve array inputs from text fields
        theta = str2num(input_texts{1}.String);
        alpha = str2num(input_texts{2}.String);
        d = str2num(input_texts{3}.String);
        a = str2num(input_texts{4}.String);
        n = length(a);
        if length(alpha) ~= n || length(d) ~= n || length(theta) ~= n
            errordlg('Input arrays must have the same length', 'Error');
            return;
        end

        % Perform forward kinematics calculation
        end_effector = forward_kinematics_mathematical(theta, alpha, d, a);
        
        % Display result in the output text box
        end_effector_str=mat2str(round(end_effector,3));
        end_effector_display= strrep(end_effector_str, ';', sprintf('\n'));
        %end_effector_display = sprintf('%0.2f\t%0.2f\t%0.2f\t%0.2f\n%0.2f\t%0.2f\t%0.2f\t%0.2f\n%0.2f\t%0.2f\t%0.2f\t%0.2f\n%0.2f\t%0.2f\t%0.2f\t%0.2f', end_effector.');
        result_text.String = sprintf('End Effector Pose:\n[%s]',(end_effector_display));
    end

    % Function to calculate inverse kinematics
    function calculate_inverse(~, ~, input_texts, result_text)
        % Array inputs from text fields
        alpha = str2num(input_texts{1}.String);
        theta_initial = str2num(input_texts{2}.String)
        a = str2num(input_texts{3}.String);
        d = str2num(input_texts{4}.String);
        end_effector_position = str2num(input_texts{5}.String);
        %check on arrays sizes
         n = length(a);
        if length(alpha) ~= n || length(d) ~= n || length(theta_initial) ~= n
            errordlg('Input arrays must have the same length', 'Error');
            return;
        end
        theta_initial = theta_initial';
        % Positions desired
        x_desired = end_effector_position(1);
        y_desired= end_effector_position(2);
        z_desired= end_effector_position(3);

        % Perform inverse kinematics calculation
        % Convergence criteria and initialization
        max_iter = 1000;
        epsilon = 1e-6;
        % Initialize the loop variables
        iter = 0;
        delta_theta = inf(3, 1);

        % Main loop for IK
        while norm(delta_theta) > epsilon && iter < max_iter
            % Forward Kinematics
            T = eye(4);
            for i = 1:n
                T_i = [cos(theta_initial(i)), -sin(theta_initial(i)) * cos(alpha(i)), sin(theta_initial(i)) * sin(alpha(i)), a(i) * cos(theta_initial(i));
                       sin(theta_initial(i)), cos(theta_initial(i)) * cos(alpha(i)), -cos(theta_initial(i)) * sin(alpha(i)), a(i) * sin(theta_initial(i));
                       0, sin(alpha(i)), cos(alpha(i)), d(i);
                       0, 0, 0, 1];
                T = T * T_i;
            end

            % Current end-effector position
            x_current = T(1, 4);
            y_current = T(2, 4);
            z_current = T(3, 4);

            % Calculate the position error
            error_x = x_desired - x_current;
            error_y = y_desired - y_current;
            error_z = z_desired - z_current;
            error = [error_x; error_y; error_z];

           % Jacobian matrix
           J = zeros(3, n);
           for i = 1:n
               % transformation matrix  to the i-th link
               T_i = eye(4);
               for j = 1:i-1
                   T_j = [cos(theta_initial(j)), -sin(theta_initial(j))*cos(alpha(j)), sin(theta_initial(j))*sin(alpha(j)), a(j)*cos(theta_initial(j));
                       sin(theta_initial(j)), cos(theta_initial(j))*cos(alpha(j)), -cos(theta_initial(j))*sin(alpha(j)), a(j)*sin(theta_initial(j));
                       0, sin(alpha(j)), cos(alpha(j)), d(j);
                       0, 0, 0, 1];
                   T_i = T_i * T_j;
               end
               % Compute the previous z-axis direction in the current frame
               z_prev = T_i(1:3, 3);
               % Compute the Jacobian column corresponding to joint i
               J(:, i) = cross(z_prev, T(1:3, 4) - T_i(1:3, 4));
           end
           
           % Pseudo-inverse of the Jacobian
           J_pseudo_inv = pinv(J);
           
           % Compute the change in joint angles
           delta_theta = J_pseudo_inv * error;
           
           % Update joint angles
           theta_initial = theta_initial + delta_theta;
           
           % Increment iteration count
           iter = iter + 1;
        end
        
        % Display the final joint angles
        fprintf('Joint angles after %d iterations:\n', iter);
        disp(theta_initial);
        if norm(delta_theta) <= epsilon
            theta_string = '[';
            for i = 1:length(theta_initial)
                theta_string = [theta_string, sprintf('%.2f', theta_initial(i))];
                if i < length(theta_initial)
                    theta_string = [theta_string, ', '];
                end
            end
            theta_string = [theta_string, ']'];
            
            result_text.String = sprintf('Converged in %d iterations.\nJoint Angles in rad: %s', iter, theta_string);
        else
            result_text.String = 'Failed to converge within the maximum iterations.';
        end
        
    end
    % Function to calculate workspace
    function calculate_workspace(~, ~, input_texts)
        % Retrieve array inputs from text fields
        alpha = str2num(input_texts{1}.String);
        a = str2num(input_texts{2}.String);
        d = str2num(input_texts{3}.String);
        max_theta = rad2deg(str2num(input_texts{4}.String));

       
        % check the lengths of entered arrays 
        if length(alpha) ~= length(a) || length(a) ~= length(d) || length(d) ~= length(max_theta)
            errordlg('Input arrays must be the same length and numerical values', 'Input Error');
            return;
        end
        
        
        % Perform workspace calculation
        plot_workspace(alpha, a, d, max_theta);
    end

    % Forward kinematics calculation
    function end_effector_pose = forward_kinematics_mathematical(theta, alpha, d, a)
        % Initialize transformation matrix
        T = eye(4);

        % Calculate forward kinematics
        for i = 1:length(a)
            A_i = [
                cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
                sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
                0, sin(alpha(i)), cos(alpha(i)), d(i);
                0, 0, 0, 1];
            T = T * A_i;
        end

        % Extract end-effector final transformation matrix
        end_effector_pose = T;
    end

    % Workspace plot function
    function plot_workspace(alpha, a, d, max_theta)
        n = length(a);

        % Create robot structure and links
        L = Link.empty(0, n);
        for i = 1:n
            L(i) = Link([0, d(i), a(i), alpha(i)]);
        end
        robot = SerialLink(L);

        % Generate random joint configurations and calculate workspace
        num_points = 2000;
        v = zeros(num_points, 3);
        for k = 1:num_points
            q = max_theta .* rand(1, n);
            T = robot.fkine(q);
            v(k, :) = transl(T);
        end

        % Get axes handle from the current figure
        hAxes = gca;

        % Clear the axes before plotting
        cla(hAxes);

        % Plot the workspace points
        scatter3(hAxes, v(:, 1), v(:, 2), v(:, 3), '*', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'r');
        
        % Set plot properties
        title(hAxes, 'Workspace');
        xlabel(hAxes, 'X');
        ylabel(hAxes, 'Y');
        zlabel(hAxes, 'Z');
        grid(hAxes, 'on');
        axis(hAxes, 'equal');
    end
end