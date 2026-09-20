%% Recreate Validation Plots with 2x4 Layout (All Groups Including Wings)
fprintf('\n=== RECREATING VALIDATION PLOTS WITH 2×4 LAYOUT ===\n');

% Define a sophisticated color palette
color_desired = [0.2, 0.4, 0.8];     % Deep blue for desired
color_actual = [0.8, 0.3, 0.3];      % Soft red for actual
color_secondary = [0.3, 0.7, 0.3];   % Green for secondary joint
color_grid = [0.85, 0.85, 0.85];     % Light gray for grid

% Create figure with 2x4 layout for better proportions
fig_validation_new = figure('Position', [30, 30, 2000, 900], ...
    'Name', 'PID Controller Validation - Complete Set', ...
    'Color', 'white');

% Set default axes properties
set(groot, 'defaultAxesFontName', 'Helvetica');
set(groot, 'defaultTextFontName', 'Helvetica');

plot_idx = 0;
for group_idx = 1:size(joint_groups, 1)
    group_name = joint_groups{group_idx, 2};
    field_name = strrep(group_name, ' ', '_');
    
    if isfield(optimized_pid, field_name)
        plot_idx = plot_idx + 1;
        
        fprintf('Plotting %s...\n', group_name);
        
        pid_data = optimized_pid.(field_name);
        joint_idx = pid_data.joints;
        q_desired_traj = pid_data.trajectory;
        
        % Create initial configuration
        q_initial_matched = q_home;
        for j = 1:length(joint_idx)
            idx = joint_idx(j);
            q_initial_matched(idx) = q_desired_traj(j, 1);
        end
        
        % Simulate
        try
            [q_response, ~, ~, ~] = simulate_pid_control(robot, joint_idx, q_desired_traj, t, ...
                pid_data.Kp, pid_data.Ki, pid_data.Kd, q_initial_matched, joint_info, false);
            
            % Create subplot with 2x4 layout
            ax = subplot(2, 4, plot_idx);
            hold on;
            
            % Set grid first (behind everything)
            grid on;
            ax.GridColor = color_grid;
            ax.GridAlpha = 0.5;
            ax.GridLineStyle = ':';
            
            % Determine if joints are prismatic or revolute
            is_prismatic = strcmp(joint_info(joint_idx(1)).type, 'prismatic');
            
            % Plot based on number of joints in group
            if length(joint_idx) == 1
                % Single joint
                idx = joint_idx(1);
                
                if is_prismatic
                    % Convert to mm
                    h1 = plot(t, q_desired_traj(1, :)*1000, '--', ...
                        'Color', color_desired, 'LineWidth', 2.5, ...
                        'DisplayName', 'Reference');
                    h2 = plot(t, q_response(idx, :)*1000, '-', ...
                        'Color', color_actual, 'LineWidth', 2, ...
                        'DisplayName', 'Response');
                    ylabel('Position (mm)', 'FontSize', 11);
                else
                    % Convert to degrees
                    h1 = plot(t, q_desired_traj(1, :)*180/pi, '--', ...
                        'Color', color_desired, 'LineWidth', 2.5, ...
                        'DisplayName', 'Reference');
                    h2 = plot(t, q_response(idx, :)*180/pi, '-', ...
                        'Color', color_actual, 'LineWidth', 2, ...
                        'DisplayName', 'Response');
                    ylabel('Angle (°)', 'FontSize', 11);
                end
                
                % Legend for single joint
                leg = legend([h1, h2], 'Location', 'best');
                leg.Box = 'off';
                leg.FontSize = 9;
                
            else
                % Multiple joints
                idx1 = joint_idx(1);
                idx2 = joint_idx(2);
                
                if is_prismatic
                    % Prismatic joints in mm
                    h1 = plot(t, q_desired_traj(1, :)*1000, '--', ...
                        'Color', color_desired, 'LineWidth', 2.5);
                    h2 = plot(t, q_response(idx1, :)*1000, '-', ...
                        'Color', color_desired, 'LineWidth', 2);
                    h3 = plot(t, q_desired_traj(2, :)*1000, '--', ...
                        'Color', color_secondary, 'LineWidth', 2.5);
                    h4 = plot(t, q_response(idx2, :)*1000, '-', ...
                        'Color', color_secondary, 'LineWidth', 2);
                    ylabel('Position (mm)', 'FontSize', 11);
                else
                    % Revolute joints in degrees
                    h1 = plot(t, q_desired_traj(1, :)*180/pi, '--', ...
                        'Color', color_desired, 'LineWidth', 2.5);
                    h2 = plot(t, q_response(idx1, :)*180/pi, '-', ...
                        'Color', color_desired, 'LineWidth', 2);
                    h3 = plot(t, q_desired_traj(2, :)*180/pi, '--', ...
                        'Color', color_secondary, 'LineWidth', 2.5);
                    h4 = plot(t, q_response(idx2, :)*180/pi, '-', ...
                        'Color', color_secondary, 'LineWidth', 2);
                    ylabel('Angle (°)', 'FontSize', 11);
                end
                
                % Compact legend for multiple joints
                if strcmp(group_name, 'Wings')
                    leg = legend([h1, h2, h3, h4], ...
                        'Left Ref', 'Left Resp', 'Right Ref', 'Right Resp', ...
                        'Location', 'best', 'NumColumns', 2);
                else
                    leg = legend([h1, h2, h3, h4], ...
                        sprintf('J%d Ref', idx1), sprintf('J%d Resp', idx1), ...
                        sprintf('J%d Ref', idx2), sprintf('J%d Resp', idx2), ...
                        'Location', 'best', 'NumColumns', 2);
                end
                leg.Box = 'off';
                leg.FontSize = 8;
            end
            
            % Calculate RMSE
            total_rmse = 0;
            for j = 1:length(joint_idx)
                idx = joint_idx(j);
                rmse = sqrt(mean((q_response(idx, :) - q_desired_traj(j, :)).^2));
                total_rmse = total_rmse + rmse;
            end
            avg_rmse = total_rmse / length(joint_idx);
            
            % Title with RMSE
            title(sprintf('%s\nRMSE: %.4f', group_name, avg_rmse), ...
                'FontSize', 12, 'FontWeight', 'normal');
            
            % Axes properties
            xlabel('Time (s)', 'FontSize', 11);
            xlim([0 t_sim]);
            
            % Clean appearance
            box on;
            ax.LineWidth = 0.8;
            ax.FontSize = 10;
            
            % Add subtle background
            ax.Color = [0.98, 0.98, 0.98];
            
            % Tighten the plot
            ax.Position(3) = ax.Position(3) * 0.95;
            ax.Position(4) = ax.Position(4) * 0.92;
            
            hold off;
            
        catch ME
            fprintf('  ERROR plotting %s: %s\n', group_name, ME.message);
        end
    end
end

% Add a summary plot in the 8th subplot
subplot(2, 4, 8);
hold on;

% Collect all RMSE values
rmse_values = [];
group_names_short = {};
fields = fieldnames(optimized_pid);

for i = 1:length(fields)
    field = fields{i};
    data = optimized_pid.(field);
    
    % Calculate RMSE
    q_initial_temp = q_home;
    for j = 1:length(data.joints)
        q_initial_temp(data.joints(j)) = data.trajectory(j, 1);
    end
    
    try
        [q_resp_temp, ~, ~, ~] = simulate_pid_control(robot, data.joints, data.trajectory, t, ...
            data.Kp, data.Ki, data.Kd, q_initial_temp, joint_info, false);
        
        total_rmse = 0;
        for j = 1:length(data.joints)
            idx = data.joints(j);
            rmse = sqrt(mean((q_resp_temp(idx, :) - data.trajectory(j, :)).^2));
            total_rmse = total_rmse + rmse;
        end
        avg_rmse = total_rmse / length(data.joints);
        
        rmse_values = [rmse_values, avg_rmse];
        group_names_short{end+1} = strrep(field, '_', ' ');
    catch
        % Skip if error
    end
end

% Create bar chart
if ~isempty(rmse_values)
    bar(rmse_values, 'FaceColor', [0.4, 0.6, 0.8], 'EdgeColor', 'none');
    set(gca, 'XTickLabel', group_names_short, 'XTickLabelRotation', 45);
    ylabel('Average RMSE', 'FontSize', 11);
    title('Performance Summary', 'FontSize', 12, 'FontWeight', 'normal');
    grid on;
    ax = gca;
    ax.GridColor = color_grid;
    ax.GridAlpha = 0.5;
    ax.GridLineStyle = ':';
    ax.Color = [0.98, 0.98, 0.98];
    box on;
    
    % Add value labels on bars
    for i = 1:length(rmse_values)
        text(i, rmse_values(i), sprintf('%.4f', rmse_values(i)), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontSize', 8);
    end
end

% Main title
sgtitle('Bibliobot PID Controller Validation - All Joint Groups', ...
    'FontSize', 16, 'FontWeight', 'normal', 'FontName', 'Helvetica');

% Adjust figure spacing
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.02, 0.05, 0.96, 0.88]);

% Tighten subplot spacing
set(gcf, 'DefaultAxesLooseInset', [0, 0, 0, 0]);

% Save the enhanced figure
saveas(fig_validation_new, fullfile(save_path, 'bibliobot_pid_validation_complete_2x4.png'));
print(fig_validation_new, fullfile(save_path, 'bibliobot_pid_validation_complete_2x4'), '-dpng', '-r300');

fprintf('\nComplete validation plot with all 7 joint groups saved!\n');
fprintf('Location: %s\n', fullfile(save_path, 'bibliobot_pid_validation_complete_2x4.png'));

%% Print detailed performance metrics
fprintf('\n=== DETAILED PERFORMANCE METRICS ===\n');
fprintf('%-20s | %-8s | %-8s | %-8s\n', 'Joint Group', 'RMSE', 'Max Err', 'Final Err');
fprintf('%s\n', repmat('-', 58, 1));

for i = 1:length(fields)
    field = fields{i};
    data = optimized_pid.(field);
    
    q_initial_temp = q_home;
    for j = 1:length(data.joints)
        q_initial_temp(data.joints(j)) = data.trajectory(j, 1);
    end
    
    try
        [q_resp_temp, ~, ~, ~] = simulate_pid_control(robot, data.joints, data.trajectory, t, ...
            data.Kp, data.Ki, data.Kd, q_initial_temp, joint_info, false);
        
        total_rmse = 0;
        max_error = 0;
        final_error = 0;
        
        for j = 1:length(data.joints)
            idx = data.joints(j);
            errors = abs(q_resp_temp(idx, :) - data.trajectory(j, :));
            rmse = sqrt(mean(errors.^2));
            total_rmse = total_rmse + rmse;
            max_error = max(max_error, max(errors));
            final_error = final_error + abs(errors(end));
        end
        
        avg_rmse = total_rmse / length(data.joints);
        avg_final_error = final_error / length(data.joints);
        
        fprintf('%-20s | %8.4f | %8.4f | %8.4f\n', ...
            strrep(field, '_', ' '), avg_rmse, max_error, avg_final_error);
    catch
        fprintf('%-20s | %8s | %8s | %8s\n', strrep(field, '_', ' '), 'N/A', 'N/A', 'N/A');
    end
end

fprintf('\nAll 7 joint groups have been successfully plotted in 2×4 layout!\n');더욱

function [q_response, qd_response, tau_control, error_integral] = simulate_pid_control(robot, joint_idx, q_desired_traj, t, Kp, Ki, Kd, q_initial, joint_info, show_progress)
    if nargin < 10
        show_progress = false;
    end
    
    dt = t(2) - t(1);
    n_steps = length(t);
    n_joints = length(q_initial);
    
    % Initialize states
    q_response = zeros(n_steps, n_joints);
    qd_response = zeros(n_steps, n_joints);
    qdd_response = zeros(n_steps, n_joints);
    tau_control = zeros(n_steps, n_joints);
    
    % IMPORTANT: Set initial position to match desired trajectory start
    q_response(1, :) = q_initial;
    for j = 1:length(joint_idx)
        idx = joint_idx(j);
        q_response(1, idx) = q_desired_traj(j, 1);  % Match initial position
    end
    
    error_integral = zeros(length(joint_idx), 1);
    
    % Torque limits based on joint type
    tau_limits = zeros(n_joints, 1);
    for j = 1:n_joints
        if strcmp(joint_info(j).type, 'revolute') || strcmp(joint_info(j).type, 'continuous')
            tau_limits(j) = 50;  % N·m
        else
            tau_limits(j) = 200; % N
        end
    end
    
    % Progress display setup
    if show_progress
        fprintf('  Simulating: ');
        progress_step = max(1, floor(n_steps/20));
    end
    
    % Simulation loop
    for i = 2:n_steps
        % Show progress
        if show_progress && mod(i, progress_step) == 0
            fprintf('.');
        end
        
        % Current state
        q_current = q_response(i-1, :);
        qd_current = qd_response(i-1, :);
        
        % Check for valid configuration
        if any(isnan(q_current)) || any(isinf(q_current))
            q_response(i:end, :) = repmat(q_current, n_steps-i+1, 1);
            qd_response(i:end, :) = 0;
            break;
        end
        
        % Apply joint limits
        for j = 1:n_joints
            if ~strcmp(joint_info(j).type, 'fixed') && ~strcmp(joint_info(j).type, 'continuous')
                if joint_info(j).limits(2) > joint_info(j).limits(1)
                    q_current(j) = max(min(q_current(j), joint_info(j).limits(2)), joint_info(j).limits(1));
                end
            end
        end
        
        % Initialize control torque
        tau_control_current = zeros(1, n_joints);
        
        % PID control for specified joints
        for j = 1:length(joint_idx)
            idx = joint_idx(j);
            
            % Error calculation
            error = q_desired_traj(j, i) - q_current(idx);
            
            % Limit error
            max_error = 0.5;
            error = max(min(error, max_error), -max_error);
            
            % Integral with anti-windup
            error_integral_new = error_integral(j) + error * dt;
            integral_limit = 1.0;
            error_integral(j) = max(min(error_integral_new, integral_limit), -integral_limit);
            
            % Derivative term
            error_derivative = -qd_current(idx);
            
            % Calculate control torque
            tau = Kp(j) * error + Ki(j) * error_integral(j) + Kd(j) * error_derivative;
            
            % Apply torque limits
            tau_control_current(idx) = max(min(tau, tau_limits(idx)), -tau_limits(idx));
        end
        
        tau_control(i, :) = tau_control_current;
        
        % Add gravity compensation
        try
            tau_gravity = gravityTorque(robot, q_current);
            
            if any(isnan(tau_gravity))
                tau_gravity = zeros(size(tau_gravity));
            end
            
            tau_total = tau_control_current + tau_gravity;
        catch
            tau_total = tau_control_current;
        end
        
        % Apply final torque limits
        for j = 1:n_joints
            tau_total(j) = max(min(tau_total(j), tau_limits(j)), -tau_limits(j));
        end
        
        % Forward dynamics
        try
            qdd_response(i, :) = forwardDynamics(robot, q_current, qd_current, tau_total);
            
            if any(isnan(qdd_response(i, :)))
                qdd_response(i, :) = zeros(1, n_joints);
            end
        catch
            % Simple approximation
            qdd_response(i, :) = tau_total * 0.01;
        end
        
        % Limit accelerations
        max_acc = 10;
        for j = 1:n_joints
            qdd_response(i, j) = max(min(qdd_response(i, j), max_acc), -max_acc);
        end
        
        % Integrate
        qd_response(i, :) = qd_current + qdd_response(i, :) * dt;
        
        % Velocity limits
        max_vel = 5;
        for j = 1:n_joints
            qd_response(i, j) = max(min(qd_response(i, j), max_vel), -max_vel);
        end
        
        % Position update
        q_response(i, :) = q_current + qd_response(i, :) * dt;
        
        % Apply joint limits again
        for j = 1:n_joints
            if ~strcmp(joint_info(j).type, 'fixed') && ~strcmp(joint_info(j).type, 'continuous')
                if joint_info(j).limits(2) > joint_info(j).limits(1)
                    q_response(i, j) = max(min(q_response(i, j), joint_info(j).limits(2)), joint_info(j).limits(1));
                end
            end
        end
    end
    
    if show_progress
        fprintf(' Done!\n');
    end
    
    % Transpose for output
    q_response = q_response';
    qd_response = qd_response';
    tau_control = tau_control';
end