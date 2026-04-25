%% 1. Simulation Execution & Time Vector Extraction
% Execute the Simulink model using the currently loaded workspace parameters
simOut = sim('unmanned_aerial_vehicle_model.slx', 'ReturnWorkspaceOutputs', 'on');
t = simOut.tout;

%% 2. Kinematic Data Extraction (Position)
% Safely parse the actual 3D position vector of the UAV.
% This conditional block handles variable Simulink output formats (timeseries vs. raw arrays) 
% and flattens potential 3D tensor outputs into a standard N x 3 matrix for evaluation.
if isprop(simOut, 'p_actual')
    raw_p = simOut.p_actual;
    if isa(raw_p, 'timeseries')
        act_p = raw_p.Data; 
    else
        act_p = raw_p; 
    end
    if ndims(act_p) == 3
        act_p = squeeze(act_p)'; 
    end
else
    disp('Error: p_actual not found. Check To Workspace block in Simulink.'); return;
end

%% 3. Control Effort Extraction (Actuator Commands)
% Parse total thrust (F_t) and commanded torques (M) to evaluate controller feasibility.
% This data proves the optimal parameters do not cause actuator saturation or chatter.
if isprop(simOut, 'F_t') && isprop(simOut, 'M')
    F_t = simOut.F_t;
    if isa(F_t, 'timeseries')
        F_t = F_t.Data; 
    end
    
    M = simOut.M;
    if isa(M, 'timeseries')
        M = M.Data; 
    end
    if ndims(M) == 3
        M = squeeze(M)'; 
    end
else
    disp('Error: F_t or M not found. Check To Workspace blocks.'); return;
end

%% 4. Analytical Trajectory Reconstruction
% Re-evaluate the reference equations using the simulation's exact time steps.
% This guarantees mathematically perfect resolution for the target plotting lines.

% ----- Four-Leaf Clover -----
%ref_x = 2*cos(t).*cos(2*t); 
%ref_y = 2*sin(t).*cos(2*t); 
%ref_z = ones(size(t)); 

% ----- Ellipse -----
%ref_x = 2*cos(t); 
%ref_y = 2*sin(t); 
%ref_z = ones(size(t));

% ----- 3D Spiral -----
ref_x = 2*cos(t); 
ref_y = 2*sin(t); 
ref_z = 0.1*t;

%% 5. Error Quantification & Steady-State Analysis
act_x = act_p(:, 1); 
act_y = act_p(:, 2); 
act_z = act_p(:, 3);

% Calculate the continuous 3D Euclidean tracking error magnitude
error_xyz = sqrt((ref_x - act_x).^2 + (ref_y - act_y).^2 + (ref_z - act_z).^2);

% Isolate steady-state performance by truncating the initial takeoff transient.
% t > 2.0s represents the flight phase after the UAV reaches the target altitude (Z=1).
steady_state_idx = find(t > 2.0);
ss_error = error_xyz(steady_state_idx);
mean_ss_error = mean(ss_error);
max_ss_error = max(ss_error);

%% 6. Visualization Suite

% --- FIGURE 1: Spatial Tracking (3D) ---
% Validates overall geometric path following capabilities.
figure('Name', '3D Trajectory', 'Color', 'w', 'Position', [50, 50, 600, 500]);
plot3(ref_x, ref_y, ref_z, 'k--', 'LineWidth', 2); hold on;
plot3(act_x, act_y, act_z, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Reference', 'HBO-PD', 'Location', 'best'); title('3D Trajectory'); view(3);

% --- FIGURE 2: Individual State Tracking (Time Domain) ---
% Diagnoses phase lag or axis-specific amplitude degradation over time.
figure('Name', 'State Tracking', 'Color', 'w', 'Position', [650, 50, 600, 600]);

subplot(3,1,1); 
plot(t, ref_x, 'k--', 'LineWidth', 2); hold on;
plot(t, act_x, 'b', 'LineWidth', 1.5);
grid on; ylabel('X (m)'); title('State Tracking');
legend('Target', 'Actual', 'Location', 'best');

subplot(3,1,2); 
plot(t, ref_y, 'k--', 'LineWidth', 2); hold on;
plot(t, act_y, 'r', 'LineWidth', 1.5);
grid on; ylabel('Y (m)');

subplot(3,1,3); 
plot(t, ref_z, 'k--', 'LineWidth', 2); hold on;
plot(t, act_z, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
grid on; ylabel('Z (m)'); xlabel('Time (s)');

% --- FIGURE 3: Error Convergence ---
% Demonstrates transient settling time and validates steady-state stability bounds.
figure('Name', 'Tracking Error', 'Color', 'w', 'Position', [50, 600, 600, 300]);
plot(t, error_xyz, 'r', 'LineWidth', 1.5); hold on;
xline(2.0, 'k--', 'Takeoff Complete', 'LabelHorizontalAlignment', 'left');
grid on; xlabel('Time (s)'); ylabel('Error (m)'); title('Euclidean Tracking Error');

% --- FIGURE 4: Actuator Control Effort ---
% Proves the optimization algorithm did not over-gain the system. 
% Verifies thrust remains positive (no free-fall) and torques do not constantly clip at maximums.
figure('Name', 'Control Effort', 'Color', 'w', 'Position', [650, 700, 600, 400]);

subplot(2,1,1);
plot(t, F_t, 'k', 'LineWidth', 1.5);
grid on; ylabel('Thrust (N)'); title('Total Thrust Command (F_t)');
yline(16.94, 'r--', 'Hover Thrust');

subplot(2,1,2);
plot(t, M(:,1), 'b', 'LineWidth', 1.2); hold on;
plot(t, M(:,2), 'r', 'LineWidth', 1.2);
plot(t, M(:,3), 'g', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Torque (N\cdotm)'); title('Attitude Control Torques');
legend('\tau_x (Roll)', '\tau_y (Pitch)', '\tau_z (Yaw)', 'Location', 'best');

%% 7. Quantitative Results Output
% Print benchmark metrics to the console for direct comparison against literature values.
fprintf('\n=== PROJECT METRICS ===\n');
fprintf('Overall Mean Error (Includes Takeoff): %.4f m\n', mean(error_xyz));
fprintf('True Steady-State Mean Error (t > 2s): %.4f m\n', mean_ss_error);
fprintf('Max Steady-State Deviation (t > 2s): %.4f m\n', max_ss_error);
fprintf('==========================================\n');