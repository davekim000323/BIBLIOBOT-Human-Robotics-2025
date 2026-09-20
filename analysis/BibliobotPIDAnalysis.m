%% Bibliobot PID Optimization with Full Range Cosine Trajectories
% Clean workspace
clear; clc; close all;

% Define save path
save_path = '/Users/mangekyo';

%% Add progress display function
global iteration_count start_time
iteration_count = 0;
start_time = tic;

% Progress display function
function stop = progress_display(x, optimValues, state)
    global iteration_count start_time
    stop = false;
    
    if strcmp(state, 'iter')
        iteration_count = iteration_count + 1;
        elapsed_time = toc(start_time);
        
        fprintf('  Iter %3d: f(x) = %.4f, Step = %.2e, Time = %.1fs\n', ...
                optimValues.iteration, optimValues.fval, optimValues.stepsize, elapsed_time);
        
        % Update every 10 iterations with more detail
        if mod(optimValues.iteration, 10) == 0
            fprintf('  --> Current best RMSE: %.4f\n', optimValues.fval);
        end
    end
end

%% 1. Load Robot Model
fprintf('=== BIBLIOBOT PID OPTIMIZATION ===\n');
fprintf('Loading Bibliobot model...\n');
robot = importrobot('Bibliobot.urdf', 'DataFormat', 'row');
robot.Gravity = [0 0 -9.81];

% Adjust mass to 36 kg total
current_total_mass = sum(arrayfun(@(i) robot.Bodies{i}.Mass, 1:robot.NumBodies));
mass_scale_factor = 36 / current_total_mass;
for i = 1:robot.NumBodies
    robot.Bodies{i}.Mass = robot.Bodies{i}.Mass * mass_scale_factor;
    robot.Bodies{i}.Inertia = robot.Bodies{i}.Inertia * mass_scale_factor;
end
fprintf('Robot mass adjusted to: %.2f kg\n', 36);

%% 2. Define Joint Groups
fprintf('\nSetting up joint groups...\n');

% Wheel joints (omni-drive)
right_wheel_connector_idx = 1;
right_wheel_idx = 2;
left_wheel_connector_idx = 3;
left_wheel_idx = 4;

% Manipulator joints
manipulator_lift_idx = 8;
manipulator_extend_idx = 9;
gripper_left_idx = 10;
gripper_right_idx = 11;
platform_idx = 12;
wing_left_idx = 13;
wing_right_idx = 14;

% Group joints by type - INCLUDING WHEELS
INCLUDE_WHEELS = true; % Now including wheels with proper trajectories

if INCLUDE_WHEELS
    fprintf('Including wheel joints in optimization\n');
    joint_groups = {
        [right_wheel_connector_idx, left_wheel_connector_idx], 'Wheel Connectors';
        [right_wheel_idx, left_wheel_idx], 'Wheels';
        manipulator_lift_idx, 'Lift';
        manipulator_extend_idx, 'Extension';
        [gripper_left_idx, gripper_right_idx], 'Grippers';
        platform_idx, 'Platform';
        [wing_left_idx, wing_right_idx], 'Wings'
    };
else
    fprintf('Excluding wheel joints from optimization\n');
    joint_groups = {
        manipulator_lift_idx, 'Lift';
        manipulator_extend_idx, 'Extension';
        [gripper_left_idx, gripper_right_idx], 'Grippers';
        platform_idx, 'Platform';
        [wing_left_idx, wing_right_idx], 'Wings'
    };
end

% Get joint info
n_joints = robot.NumBodies;
joint_info = struct();
for i = 1:robot.NumBodies
    joint = robot.Bodies{i}.Joint;
    joint_info(i).name = joint.Name;
    joint_info(i).type = joint.Type;
    joint_info(i).body = robot.Bodies{i}.Name;
    if ~strcmp(joint.Type, 'fixed')
        joint_info(i).limits = joint.PositionLimits;
    else
        joint_info(i).limits = [0, 0];
    end
end

%% 3. PID Controller Function with Progress
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

%% 4. Objective Function with Initial Position Matching
global objective_call_count
objective_call_count = 0;

function rmse = pid_objective(pid_params, robot, joint_idx, q_desired_traj, t, q_initial_full, joint_info)
    global objective_call_count
    objective_call_count = objective_call_count + 1;
    
    % Only show progress every 5 calls
    if mod(objective_call_count, 5) == 0
        fprintf('    [Objective evaluation #%d]', objective_call_count);
    end
    
    % Input validation
    if any(isnan(pid_params)) || any(isinf(pid_params))
        rmse = 1e6;
        return;
    end
    
    % Extract PID gains
    n_joints = length(joint_idx);
    if length(pid_params) ~= 3 * n_joints
        rmse = 1e6;
        return;
    end
    
    Kp = pid_params(1:n_joints);
    Ki = pid_params(n_joints+1:2*n_joints);
    Kd = pid_params(2*n_joints+1:3*n_joints);
    
    % Minimum gain check
    if any(Kp < 0.01)
        rmse = 1e6;
        return;
    end
    
    % Create initial configuration that matches trajectory start
    q_initial_matched = q_initial_full;
    for j = 1:length(joint_idx)
        idx = joint_idx(j);
        q_initial_matched(idx) = q_desired_traj(j, 1);
    end
    
    % Simulate
    try
        show_prog = mod(objective_call_count, 5) == 0;
        [q_response, ~, ~, ~] = simulate_pid_control(robot, joint_idx, q_desired_traj, t, Kp, Ki, Kd, q_initial_matched, joint_info, show_prog);
        
        % Calculate RMSE
        error = 0;
        for j = 1:length(joint_idx)
            idx = joint_idx(j);
            error = error + mean((q_response(idx, :) - q_desired_traj(j, :)).^2);
        end
        rmse = sqrt(error / length(joint_idx));
        
        if isnan(rmse) || isinf(rmse)
            rmse = 1e6;
        end
        
        if mod(objective_call_count, 5) == 0
            fprintf(' RMSE = %.4f\n', rmse);
        end
        
    catch
        rmse = 1e6;
        if mod(objective_call_count, 5) == 0
            fprintf(' Failed\n');
        end
    end
end

%% 5. Optimize PID for Each Joint Group
% Simulation parameters
t_sim = 3;
dt = 0.001;
t = 0:dt:t_sim;
n_steps = length(t);

fprintf('\nSimulation parameters:\n');
fprintf('  Duration: %.1f seconds\n', t_sim);
fprintf('  Time step: %.3f seconds\n', dt);
fprintf('  Total steps: %d\n', n_steps);

% Store optimized PID gains
optimized_pid = struct();

% Home configuration
q_home = homeConfiguration(robot);

fprintf('\n=== PID OPTIMIZATION FOR EACH JOINT GROUP ===\n');
fprintf('Total groups to optimize: %d\n', size(joint_groups, 1));

total_start_time = tic;

for group_idx = 1:size(joint_groups, 1)
    joint_idx = joint_groups{group_idx, 1};
    group_name = joint_groups{group_idx, 2};
    
    fprintf('\n[GROUP %d/%d] Optimizing %s...\n', group_idx, size(joint_groups, 1), group_name);
    fprintf('Joint indices: ');
    fprintf('%d ', joint_idx);
    fprintf('\n');
    
    % Reset counters
    global iteration_count objective_call_count
    iteration_count = 0;
    objective_call_count = 0;
    start_time = tic;
    
    % Generate test trajectory - PURE COSINE FOR ALL JOINTS
    fprintf('Generating full-range cosine trajectory...\n');
    q_desired_traj = zeros(length(joint_idx), n_steps);
    
    for j = 1:length(joint_idx)
        idx = joint_idx(j);
        
        % Special handling for wheels - rotation and return
        if ismember(idx, [right_wheel_idx, left_wheel_idx, right_wheel_connector_idx, left_wheel_connector_idx])
            % For wheels: rotate pi and return to original position
            center = 0;  % Wheels typically start at 0
            
            % Create trajectory: 0 -> pi -> 0
            phase1_end = floor(n_steps/2);
            phase2_end = n_steps;
            
            % Phase 1: Rotate to pi
            for k = 1:phase1_end
                progress = k / phase1_end;
                q_desired_traj(j, k) = center + pi * 0.5 * (1 - cos(pi * progress));
            end
            
            % Phase 2: Return to original
            for k = (phase1_end+1):phase2_end
                progress = (k - phase1_end) / (phase2_end - phase1_end);
                q_desired_traj(j, k) = center + pi - pi * 0.5 * (1 - cos(pi * progress));
            end
            
            % Set initial position
            q_home(idx) = center;
            
            fprintf('  Joint %d (Wheel): rotation pi and return, starting from %.3f\n', idx, center);
            
        else
            % For ALL other joints: PURE COSINE using FULL RANGE
            range = joint_info(idx).limits;
            
            if strcmp(joint_info(idx).type, 'continuous')
                % Continuous joints without limits
                center = 0;
                amplitude = pi/2;  % Reasonable range for continuous joints
                min_pos = center - amplitude;
                max_pos = center + amplitude;
            elseif range(2) > range(1)
                % Joints with valid limits - USE FULL RANGE
                min_pos = range(1);
                max_pos = range(2);
                center = (min_pos + max_pos) / 2;
                amplitude = (max_pos - min_pos) / 2;
            else
                % Fallback for joints without proper limits
                center = 0;
                amplitude = pi/4;
                min_pos = center - amplitude;
                max_pos = center + amplitude;
            end
            
            % Generate pure cosine trajectory: max -> center -> min -> center -> max
            q_desired_traj(j, :) = center + amplitude * cos(2*pi*t/t_sim);
            
            % Set initial position to maximum (cos(0) = 1)
            initial_pos = max_pos;
            q_home(idx) = initial_pos;
            
            fprintf('  Joint %d: Full range [%.3f, %.3f], amplitude=%.3f, starting at max=%.3f\n', ...
                    idx, min_pos, max_pos, amplitude, initial_pos);
        end
    end
    
    % Create initial configuration matching trajectory start
    q_initial_matched = q_home;
    
    % Initial PID guess - adjusted for full range motion
    fprintf('Setting initial PID values for full range motion...\n');
    Kp_init = zeros(1, length(joint_idx));
    Ki_init = zeros(1, length(joint_idx));
    Kd_init = zeros(1, length(joint_idx));
    
    for j = 1:length(joint_idx)
        idx = joint_idx(j);
        
        if strcmp(joint_info(idx).type, 'continuous') || strcmp(joint_info(idx).type, 'revolute')
            % Rotational joints
            if ismember(idx, [right_wheel_idx, left_wheel_idx, right_wheel_connector_idx, left_wheel_connector_idx])
                % Wheels need higher gains
                Kp_init(j) = 100;  % Increased for full range
                Ki_init(j) = 10;
                Kd_init(j) = 5;
            else
                Kp_init(j) = 50;   % Increased for full range
                Ki_init(j) = 5;
                Kd_init(j) = 2;
            end
        else
            % Prismatic joints
            if idx == manipulator_lift_idx
                Kp_init(j) = 200;  % Increased for full range
                Ki_init(j) = 20;
                Kd_init(j) = 10;
            elseif idx == manipulator_extend_idx
                Kp_init(j) = 150;  % Increased for full range
                Ki_init(j) = 15;
                Kd_init(j) = 5;
            else
                Kp_init(j) = 100;  % Increased for full range
                Ki_init(j) = 10;
                Kd_init(j) = 5;
            end
        end
    end
    
    % Combine into single vector
    initial_gains = [Kp_init, Ki_init, Kd_init];
    
    % Display initial gains
    fprintf('  Initial gains structure:\n');
    fprintf('    Kp: ['); fprintf('%.1f ', Kp_init); fprintf(']\n');
    fprintf('    Ki: ['); fprintf('%.1f ', Ki_init); fprintf(']\n');
    fprintf('    Kd: ['); fprintf('%.1f ', Kd_init); fprintf(']\n');
    
    % Optimization bounds - increased for full range
    n_params = 3 * length(joint_idx);
    lb = ones(1, n_params) * 0;
    ub = [1000*ones(1, length(joint_idx)), ...  % Increased upper bounds
          100*ones(1, length(joint_idx)), ...
          50*ones(1, length(joint_idx))];
    
    % Optimization options
    options = optimoptions('fmincon', ...
        'Display', 'iter-detailed', ...
        'MaxIterations', 50, ...
        'OptimalityTolerance', 1e-4, ...
        'StepTolerance', 1e-4, ...
        'OutputFcn', @progress_display);
    
    % Test objective function
    fprintf('Testing initial configuration...\n');
    try
        test_rmse = pid_objective(initial_gains, robot, joint_idx, q_desired_traj, t, q_home, joint_info);
        fprintf('Initial RMSE: %.4f\n', test_rmse);
        
        if test_rmse >= 1e6
            error('Invalid initial point');
        end
    catch ME
        fprintf('ERROR: Skipping %s: %s\n', group_name, ME.message);
        
        % Use default gains
        n_joints = length(joint_idx);
        Kp_opt = initial_gains(1:n_joints);
        Ki_opt = initial_gains(n_joints+1:2*n_joints);
        Kd_opt = initial_gains(2*n_joints+1:3*n_joints);
        optimized_pid.(strrep(group_name, ' ', '_')) = struct('Kp', Kp_opt, 'Ki', Ki_opt, 'Kd', Kd_opt, 'joints', joint_idx);
        continue;
    end
    
    % Run optimization
    fprintf('Starting optimization...\n');
    objective_fun = @(x) pid_objective(x, robot, joint_idx, q_desired_traj, t, q_home, joint_info);
    
    try
        optimized_params = fmincon(objective_fun, initial_gains, [], [], [], [], lb, ub, [], options);
        
        % Extract optimized gains
        n_joints = length(joint_idx);
        Kp_opt = optimized_params(1:n_joints);
        Ki_opt = optimized_params(n_joints+1:2*n_joints);
        Kd_opt = optimized_params(2*n_joints+1:3*n_joints);
        
        fprintf('\nOptimization completed in %.1f seconds\n', toc(start_time));
        fprintf('Total objective evaluations: %d\n', objective_call_count);
        fprintf('Optimized PID gains for %s:\n', group_name);
        for j = 1:length(joint_idx)
            fprintf('  Joint %d: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', ...
                    joint_idx(j), Kp_opt(j), Ki_opt(j), Kd_opt(j));
        end
    catch ME
        fprintf('ERROR: Optimization failed for %s: %s\n', group_name, ME.message);
        % Use initial gains
        n_joints = length(joint_idx);
        Kp_opt = initial_gains(1:n_joints);
        Ki_opt = initial_gains(n_joints+1:2*n_joints);
        Kd_opt = initial_gains(2*n_joints+1:3*n_joints);
    end
    
    % Store results
    optimized_pid.(strrep(group_name, ' ', '_')) = struct('Kp', Kp_opt, 'Ki', Ki_opt, 'Kd', Kd_opt, 'joints', joint_idx, 'trajectory', q_desired_traj);
end

fprintf('\n=== OPTIMIZATION COMPLETE ===\n');
fprintf('Total time: %.1f seconds\n', toc(total_start_time));

%% 6. Validate Optimized PID Controllers
fprintf('\n=== VALIDATING OPTIMIZED PID CONTROLLERS ===\n');

% Create validation figure
fig_validation = figure('Position', [100, 100, 1600, 900], 'Name', 'PID Validation Results - Full Range');

plot_idx = 0;
for group_idx = 1:size(joint_groups, 1)
    group_name = joint_groups{group_idx, 2};
    field_name = strrep(group_name, ' ', '_');
    
    if isfield(optimized_pid, field_name)
        plot_idx = plot_idx + 1;
        if plot_idx > 6
            break;
        end
        
        fprintf('\nValidating %s...\n', group_name);
        
        pid_data = optimized_pid.(field_name);
        joint_idx = pid_data.joints;
        
        % Use stored trajectory
        q_desired_traj = pid_data.trajectory;
        
        % Create initial configuration matching trajectory
        q_initial_matched = q_home;
        for j = 1:length(joint_idx)
            idx = joint_idx(j);
            q_initial_matched(idx) = q_desired_traj(j, 1);
        end
        
        % Simulate with optimized PID
        fprintf('  Running validation simulation...\n');
        try
            [q_response, qd_response, tau_control, ~] = simulate_pid_control(robot, joint_idx, q_desired_traj, t, ...
                                                                             pid_data.Kp, pid_data.Ki, pid_data.Kd, q_initial_matched, joint_info, true);
            
            % Plot results
            subplot(2, 3, plot_idx);
            hold on; grid on; box on;
            
            % Plot desired and actual trajectories
            colors = lines(length(joint_idx));
            for j = 1:length(joint_idx)
                idx = joint_idx(j);
                if strcmp(joint_info(idx).type, 'prismatic')
                    plot(t, q_desired_traj(j, :)*1000, '--', 'Color', colors(j, :), 'LineWidth', 1.5);
                    plot(t, q_response(idx, :)*1000, '-', 'Color', colors(j, :), 'LineWidth', 2);
                    ylabel('Position (mm)');
                else
                    plot(t, q_desired_traj(j, :)*180/pi, '--', 'Color', colors(j, :), 'LineWidth', 1.5);
                    plot(t, q_response(idx, :)*180/pi, '-', 'Color', colors(j, :), 'LineWidth', 2);
                    ylabel('Angle (deg)');
                end
            end
            
            xlabel('Time (s)');
            title(sprintf('%s - PID Tracking', group_name));
            legend('Desired', 'Actual', 'Location', 'best');
            xlim([0 t_sim]);
            set(gca, 'GridAlpha', 0.3);
            
            % Calculate and display RMSE
            total_rmse = 0;
            for j = 1:length(joint_idx)
                idx = joint_idx(j);
                rmse = sqrt(mean((q_response(idx, :) - q_desired_traj(j, :)).^2));
                fprintf('  Joint %d RMSE: %.4f\n', joint_idx(j), rmse);
                total_rmse = total_rmse + rmse;
            end
            fprintf('  Average RMSE: %.4f\n', total_rmse / length(joint_idx));
            
            % Add RMSE to plot title
            title(sprintf('%s - Avg RMSE: %.4f', group_name, total_rmse / length(joint_idx)));
            
        catch ME
            fprintf('  ERROR: Validation failed: %s\n', ME.message);
        end
    end
end

sgtitle('PID Controller Validation - Full Range Cosine Trajectories', 'FontSize', 14, 'FontWeight', 'normal');
saveas(fig_validation, fullfile(save_path, 'bibliobot_pid_validation_full_range.png'));

%% 7. Save Optimized PID Gains
save(fullfile(save_path, 'bibliobot_optimized_pid_full_range.mat'), 'optimized_pid');
fprintf('\n=== ALL TASKS COMPLETE ===\n');
fprintf('Optimized gains saved to: %s\n', fullfile(save_path, 'bibliobot_optimized_pid_full_range.mat'));
fprintf('Validation plot saved to: %s\n', fullfile(save_path, 'bibliobot_pid_validation_full_range.png'));

%% 8. Display Summary
fprintf('\n=== OPTIMIZATION SUMMARY ===\n');
fields = fieldnames(optimized_pid);
for i = 1:length(fields)
    field = fields{i};
    data = optimized_pid.(field);
    fprintf('\n%s:\n', strrep(field, '_', ' '));
    for j = 1:length(data.joints)
        fprintf('  Joint %d: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', ...
                data.joints(j), data.Kp(j), data.Ki(j), data.Kd(j));
    end
end

fprintf('\n=== KEY IMPROVEMENTS ===\n');
fprintf('1. All joints use FULL range cosine trajectories\n');
fprintf('2. Initial position set to maximum value (cos(0) = 1)\n');
fprintf('3. No transition phase - pure cosine from start to finish\n');
fprintf('4. Increased initial PID gains for full range motion\n');
fprintf('5. Higher optimization bounds to handle challenging trajectories\n');