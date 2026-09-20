%% Bibliobot PID Tuning - Coarse-to-Fine Grid Search (Final Version)
clear; clc; close all;

% Define save path
save_path = '/Users/mangekyo';

%% 1. Load Robot Model
fprintf('=== BIBLIOBOT PID TUNING - COARSE-TO-FINE METHOD ===\n');
fprintf('Loading Bibliobot model...\n');
robot = importrobot('Bibliobot.urdf', 'DataFormat', 'row');
robot.Gravity = [0 0 -9.81];

% Adjust mass to 36 kg
current_total_mass = sum(arrayfun(@(i) robot.Bodies{i}.Mass, 1:robot.NumBodies));
mass_scale_factor = 36 / current_total_mass;
for i = 1:robot.NumBodies
    robot.Bodies{i}.Mass = robot.Bodies{i}.Mass * mass_scale_factor;
    robot.Bodies{i}.Inertia = robot.Bodies{i}.Inertia * mass_scale_factor;
end
fprintf('Robot mass adjusted to: %.2f kg\n', 36);

%% 2. Define Manipulator Joints Only (No Wheels)
manipulator_joints = struct();
manipulator_joints.lift = 8;
manipulator_joints.extend = 9;
manipulator_joints.gripper_left = 10;
manipulator_joints.gripper_right = 11;
manipulator_joints.platform = 12;
manipulator_joints.wing_left = 13;
manipulator_joints.wing_right = 14;

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

% Home configuration
q_home = homeConfiguration(robot);

%% 3. Cosine Tracking Evaluation Function
function [rmse, max_error, avg_delay, response] = evaluate_cosine_tracking(robot, joint_idx, Kp, Ki, Kd, joint_info, q_home, t_sim, dt)
    if nargin < 8
        t_sim = 1.5;
        dt = 0.005;
    end
    
    t = 0:dt:t_sim;
    n_steps = length(t);
    n_joints = length(q_home);
    
    % Generate cosine trajectory
    if strcmp(joint_info(joint_idx).type, 'prismatic')
        if joint_info(joint_idx).limits(2) > joint_info(joint_idx).limits(1)
            range = joint_info(joint_idx).limits;
            center = (range(1) + range(2)) / 2;
            amplitude = (range(2) - range(1)) * 0.3;
        else
            center = 0;
            amplitude = 0.05;
        end
    else
        if joint_info(joint_idx).limits(2) > joint_info(joint_idx).limits(1)
            range = joint_info(joint_idx).limits;
            center = (range(1) + range(2)) / 2;
            amplitude = (range(2) - range(1)) * 0.3;
        else
            center = 0;
            amplitude = pi/4;
        end
    end
    
    q_desired = center + amplitude * cos(2*pi*t/t_sim);
    
    % Initialize
    q = zeros(n_steps, n_joints);
    qd = zeros(n_steps, n_joints);
    q(1, :) = q_home;
    
    % PID simulation
    error_integral = 0;
    tau_limit = 100;
    
    for i = 2:n_steps
        % PID control
        error = q_desired(i) - q(i-1, joint_idx);
        error_integral = error_integral + error * dt;
        error_integral = max(min(error_integral, 1), -1);
        error_derivative = -qd(i-1, joint_idx);
        
        tau_control = zeros(1, n_joints);
        tau_pid = Kp * error + Ki * error_integral + Kd * error_derivative;
        tau_control(joint_idx) = max(min(tau_pid, tau_limit), -tau_limit);
        
        % Gravity compensation
        try
            tau_gravity = gravityTorque(robot, q(i-1, :));
            tau_total = tau_control + tau_gravity;
        catch
            tau_total = tau_control;
        end
        
        % Simple dynamics
        qdd = tau_total * 0.01;
        qdd = max(min(qdd, 10), -10);
        
        % Integrate
        qd(i, :) = qd(i-1, :) + qdd * dt;
        q(i, :) = q(i-1, :) + qd(i, :) * dt;
        
        % Apply joint limits
        if ~strcmp(joint_info(joint_idx).type, 'continuous')
            if joint_info(joint_idx).limits(2) > joint_info(joint_idx).limits(1)
                q(i, joint_idx) = max(min(q(i, joint_idx), joint_info(joint_idx).limits(2)), ...
                                     joint_info(joint_idx).limits(1));
            end
        end
    end
    
    % Calculate metrics
    response = q(:, joint_idx);
    
    % RMSE
    rmse = sqrt(mean((response - q_desired').^2));
    
    % Maximum error
    max_error = max(abs(response - q_desired'));
    
    % Average phase delay
    [~, desired_peaks] = findpeaks(q_desired);
    [~, response_peaks] = findpeaks(response);
    if length(response_peaks) >= 2 && length(desired_peaks) >= 2
        delays = (response_peaks(1:min(2, length(response_peaks))) - ...
                 desired_peaks(1:min(2, length(desired_peaks)))) * dt;
        avg_delay = mean(abs(delays));
    else
        avg_delay = 0.5;
    end
end

%% 4. Grid Search Function
function [best_params, all_results, top5] = grid_search(robot, joint_id, grid, joint_info, q_home, verbose)
    if nargin < 6
        verbose = true;
    end
    
    results = [];
    total = length(grid.Kp) * length(grid.Ki) * length(grid.Kd);
    count = 0;
    
    if verbose
        fprintf('Testing %d combinations: ', total);
    end
    
    for kp = grid.Kp
        for ki = grid.Ki
            for kd = grid.Kd
                count = count + 1;
                if verbose && mod(count, max(1, floor(total/20))) == 0
                    fprintf('.');
                end
                
                try
                    [rmse, max_error, avg_delay] = evaluate_cosine_tracking(...
                        robot, joint_id, kp, ki, kd, joint_info, q_home);
                    
                    score = rmse * 100 + max_error * 20 + avg_delay * 50;
                    results(end+1, :) = [kp, ki, kd, rmse, max_error, avg_delay, score];
                catch
                    results(end+1, :) = [kp, ki, kd, 1, 1, 1, 1000];
                end
            end
        end
    end
    
    if verbose
        fprintf(' Done!\n');
    end
    
    % Sort by score
    [~, idx] = sort(results(:, 7));
    all_results = results;
    top5 = results(idx(1:min(5, size(results, 1))), :);
    best_params = top5(1, 1:3);
end

%% 5. Coarse-to-Fine Tuning Function
function [best_pid, tuning_history] = coarse_to_fine_tuning(robot, joint_id, joint_info, q_home, joint_name)
    fprintf('\n=== Coarse-to-Fine Tuning: %s (Joint %d) ===\n', joint_name, joint_id);
    
    tuning_history = struct();
    
    % Phase 1: Coarse search
    fprintf('Phase 1: Coarse search\n');
    
    if strcmp(joint_info(joint_id).type, 'prismatic')
        coarse_grid.Kp = [25, 50, 100, 200, 400, 800];
        coarse_grid.Ki = [2, 5, 10, 20, 40, 80];
        coarse_grid.Kd = [1, 2, 5, 10, 20, 40];
    else
        coarse_grid.Kp = [5, 10, 20, 40, 80, 160];
        coarse_grid.Ki = [0.5, 1, 2, 5, 10, 20];
        coarse_grid.Kd = [0.25, 0.5, 1, 2, 5, 10];
    end
    
    tic;
    [best_coarse, coarse_results, coarse_top5] = grid_search(robot, joint_id, coarse_grid, joint_info, q_home);
    coarse_time = toc;
    
    fprintf('  Best coarse: Kp=%.1f, Ki=%.2f, Kd=%.2f (%.1f seconds)\n', ...
            best_coarse(1), best_coarse(2), best_coarse(3), coarse_time);
    
    tuning_history.coarse_grid = coarse_grid;
    tuning_history.coarse_results = coarse_results;
    tuning_history.coarse_top5 = coarse_top5;
    tuning_history.coarse_time = coarse_time;
    
    % Phase 2: Fine search around best region
    fprintf('Phase 2: Fine search around best region\n');
    
    % Create fine grid around best coarse value
    fine_range = 0.5; % Search ±50% around best
    fine_points = 7;  % Number of points in each dimension
    
    fine_grid.Kp = linspace(best_coarse(1)*(1-fine_range), best_coarse(1)*(1+fine_range), fine_points);
    fine_grid.Ki = linspace(best_coarse(2)*(1-fine_range), best_coarse(2)*(1+fine_range), fine_points-2);
    fine_grid.Kd = linspace(best_coarse(3)*(1-fine_range), best_coarse(3)*(1+fine_range), fine_points-2);
    
    % Ensure positive values and remove duplicates
    fine_grid.Kp = unique(max(0.1, round(fine_grid.Kp, 1)));
    fine_grid.Ki = unique(max(0.01, round(fine_grid.Ki, 2)));
    fine_grid.Kd = unique(max(0.01, round(fine_grid.Kd, 2)));
    
    tic;
    [best_fine, fine_results, fine_top5] = grid_search(robot, joint_id, fine_grid, joint_info, q_home);
    fine_time = toc;
    
    fprintf('  Best fine: Kp=%.1f, Ki=%.2f, Kd=%.2f (%.1f seconds)\n', ...
            best_fine(1), best_fine(2), best_fine(3), fine_time);
    
    tuning_history.fine_grid = fine_grid;
    tuning_history.fine_results = fine_results;
    tuning_history.fine_top5 = fine_top5;
    tuning_history.fine_time = fine_time;
    
    % Phase 3: Ultra-fine search (optional, only if score improved significantly)
    if fine_top5(1, 7) < coarse_top5(1, 7) * 0.8  % 20% improvement
        fprintf('Phase 3: Ultra-fine search (significant improvement detected)\n');
        
        ultra_range = 0.2;  % ±20%
        ultra_points = 5;
        
        ultra_grid.Kp = linspace(best_fine(1)*(1-ultra_range), best_fine(1)*(1+ultra_range), ultra_points);
        ultra_grid.Ki = linspace(best_fine(2)*(1-ultra_range), best_fine(2)*(1+ultra_range), ultra_points);
        ultra_grid.Kd = linspace(best_fine(3)*(1-ultra_range), best_fine(3)*(1+ultra_range), ultra_points);
        
        ultra_grid.Kp = unique(max(0.1, round(ultra_grid.Kp, 1)));
        ultra_grid.Ki = unique(max(0.01, round(ultra_grid.Ki, 2)));
        ultra_grid.Kd = unique(max(0.01, round(ultra_grid.Kd, 2)));
        
        tic;
        [best_ultra, ultra_results, ultra_top5] = grid_search(robot, joint_id, ultra_grid, joint_info, q_home);
        ultra_time = toc;
        
        fprintf('  Best ultra: Kp=%.1f, Ki=%.2f, Kd=%.2f (%.1f seconds)\n', ...
                best_ultra(1), best_ultra(2), best_ultra(3), ultra_time);
        
        tuning_history.ultra_grid = ultra_grid;
        tuning_history.ultra_results = ultra_results;
        tuning_history.ultra_top5 = ultra_top5;
        tuning_history.ultra_time = ultra_time;
        
        best_pid = best_ultra;
    else
        best_pid = best_fine;
    end
    
    % Summary
    fprintf('  Total tuning time: %.1f seconds\n', sum([coarse_time, fine_time]));
    fprintf('  Total combinations tested: %d\n', ...
            size(coarse_results, 1) + size(fine_results, 1) + ...
            (isfield(tuning_history, 'ultra_results') * size(tuning_history.ultra_results, 1)));
    
    % Show top 5 final results
    if isfield(tuning_history, 'ultra_top5')
        final_top5 = tuning_history.ultra_top5;
    else
        final_top5 = fine_top5;
    end
    
    fprintf('\n  Top 5 PID combinations:\n');
    fprintf('  Rank | Kp     Ki     Kd   | RMSE    MaxErr  Delay   | Score\n');
    fprintf('  -----|------------------|------------------------|--------\n');
    for i = 1:min(5, size(final_top5, 1))
        fprintf('   %d   | %5.1f  %5.2f  %5.2f | %.4f  %.4f  %.4f | %.2f\n', ...
                i, final_top5(i, 1:3), final_top5(i, 4:6), final_top5(i, 7));
    end
end

%% 6. Main Tuning Process
fprintf('\n=== STARTING COARSE-TO-FINE PID TUNING ===\n');

joint_names = fieldnames(manipulator_joints);
tuning_results = struct();
best_gains = struct();

total_start_time = tic;

for j_idx = 1:length(joint_names)
    joint_name = joint_names{j_idx};
    joint_id = manipulator_joints.(joint_name);
    
    % Run coarse-to-fine tuning
    [best_pid, history] = coarse_to_fine_tuning(robot, joint_id, joint_info, q_home, joint_name);
    
    % Store results
    tuning_results.(joint_name).best_pid = best_pid;
    tuning_results.(joint_name).history = history;
    best_gains.(joint_name) = struct('Kp', best_pid(1), 'Ki', best_pid(2), 'Kd', best_pid(3));
end

total_time = toc(total_start_time);

%% 7. Summary
fprintf('\n=== FINAL SUMMARY ===\n');
fprintf('Total tuning time: %.1f seconds\n\n', total_time);

fprintf('Joint            | Best PID Gains          | Final Score\n');
fprintf('-----------------|-------------------------|------------\n');

for j_idx = 1:length(joint_names)
    joint_name = joint_names{j_idx};
    best = tuning_results.(joint_name).best_pid;
    
    % Get final score
    if isfield(tuning_results.(joint_name).history, 'ultra_top5')
        score = tuning_results.(joint_name).history.ultra_top5(1, 7);
    else
        score = tuning_results.(joint_name).history.fine_top5(1, 7);
    end
    
    fprintf('%-15s  | Kp=%5.1f Ki=%5.2f Kd=%5.2f | %.2f\n', ...
            joint_name, best(1), best(2), best(3), score);
end

%% 8. Validation and Visualization
fprintf('\n=== VALIDATION ===\n');

fig = figure('Position', [100, 100, 1600, 900]);
sgtitle('PID Tuning Results - Coarse-to-Fine Method', 'FontSize', 14);

% Longer validation simulation
val_t_sim = 3;
val_dt = 0.002;
val_t = 0:val_dt:val_t_sim;

for j_idx = 1:min(6, length(joint_names))
    joint_name = joint_names{j_idx};
    joint_id = manipulator_joints.(joint_name);
    best_pid = tuning_results.(joint_name).best_pid;
    
    subplot(2, 3, j_idx);
    
    % Generate validation trajectory
    if strcmp(joint_info(joint_id).type, 'prismatic')
        if joint_info(joint_id).limits(2) > joint_info(joint_id).limits(1)
            range = joint_info(joint_id).limits;
            center = (range(1) + range(2)) / 2;
            amplitude = (range(2) - range(1)) * 0.3;
        else
            center = 0;
            amplitude = 0.05;
        end
    else
        if joint_info(joint_id).limits(2) > joint_info(joint_id).limits(1)
            range = joint_info(joint_id).limits;
            center = (range(1) + range(2)) / 2;
            amplitude = (range(2) - range(1)) * 0.3;
        else
            center = 0;
            amplitude = pi/4;
        end
    end
    
    q_desired = center + amplitude * cos(2*pi*val_t/val_t_sim);
    
    % Simulate with best PID
    [rmse, max_err, delay, response] = evaluate_cosine_tracking(...
        robot, joint_id, best_pid(1), best_pid(2), best_pid(3), joint_info, q_home, val_t_sim, val_dt);
    
    % Plot
    hold on;
    if strcmp(joint_info(joint_id).type, 'prismatic')
        plot(val_t, q_desired*1000, 'b--', 'LineWidth', 2);
        plot(val_t, response*1000, 'r-', 'LineWidth', 1.5);
        ylabel('Position (mm)');
    else
        plot(val_t, q_desired*180/pi, 'b--', 'LineWidth', 2);
        plot(val_t, response*180/pi, 'r-', 'LineWidth', 1.5);
        ylabel('Angle (deg)');
    end
    
    xlabel('Time (s)');
    title(sprintf('%s\nKp=%.1f, Ki=%.2f, Kd=%.2f\nRMSE=%.4f', ...
                  joint_name, best_pid(1), best_pid(2), best_pid(3), rmse));
    legend('Desired', 'Actual', 'Location', 'best');
    grid on;
    xlim([0 val_t_sim]);
end

%% 9. Save Results
save_data = struct();
save_data.tuning_results = tuning_results;
save_data.best_gains = best_gains;
save_data.total_time = total_time;
save_data.joint_info = joint_info;

save(fullfile(save_path, 'bibliobot_coarse_fine_pid.mat'), 'save_data');
saveas(fig, fullfile(save_path, 'bibliobot_coarse_fine_pid_results.png'));

fprintf('\n=== TUNING COMPLETE ===\n');
fprintf('Results saved to: %s\n', fullfile(save_path, 'bibliobot_coarse_fine_pid.mat'));
fprintf('Plot saved to: %s\n', fullfile(save_path, 'bibliobot_coarse_fine_pid_results.png'));

%% 10. Export Best Gains for Easy Use
fprintf('\n=== COPY-PASTE READY PID GAINS ===\n');
fprintf('%%---- Copy below for your controller ----%%\n');
for j_idx = 1:length(joint_names)
    joint_name = joint_names{j_idx};
    joint_id = manipulator_joints.(joint_name);
    best = best_gains.(joint_name);
    
    fprintf('pid_gains(%d) = struct(''Kp'', %.1f, ''Ki'', %.2f, ''Kd'', %.2f); %% %s\n', ...
            joint_id, best.Kp, best.Ki, best.Kd, joint_name);
end
fprintf('%%----------------------------------------%%\n');