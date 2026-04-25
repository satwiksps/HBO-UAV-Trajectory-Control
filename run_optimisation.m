warning('off', 'all');

%% 1. Optimization Hyperparameters and Controller Bounds
% Set iteration counts for the initial exploration and exploitation phases
num_init = 10;
num_iters_phase1 = 30;
num_iters_phase2 = 30;

% Weighting factor prioritizing position error (e_p) over angular error (e_psi)
alpha_w = 1.0;

% Define parameter search space [Lower Bound, Upper Bound]
% Sequence: Kp_pos(x,y,z), Kd_pos(x,y,z), Kp_ang(roll,pitch,yaw), Kd_ang(roll,pitch,yaw)
% Bounds are tightly constrained to prevent actuator saturation and system chatter
bounds = [
    1.0 12.0; 1.0 12.0; 1.0 15.0;  
    1.0 8.0; 1.0 8.0; 1.0 10.0;  
    0.01 0.2; 0.01 0.2; 0.01 0.3;  
    0.001 0.02; 0.001 0.02; 0.001 0.02 
];

%% 2. Bootstrap Phase: Initial Data Sampling
% Preallocate training matrices for computational efficiency
X_train = zeros(num_init, 12);
e_train = zeros(num_init, 1);

for i = 1:num_init
    % Generate uniformly distributed random samples within the specified bounds
    X_train(i,:) = bounds(:,1)' + rand(1,12) .* (bounds(:,2)' - bounds(:,1)');
    
    % Inject candidate PD gains directly into the MATLAB base workspace for Simulink
    assignin('base', 'Kp_pos', X_train(i,1:3)');
    assignin('base', 'Kd_pos', X_train(i,4:6)');
    assignin('base', 'Kp_ang', X_train(i,7:9)');
    assignin('base', 'Kd_ang', X_train(i,10:12)');
    
    % Execute the nonlinear 6-DOF quadrotor physics model
    simOut = sim('unmanned_aerial_vehicle_model.slx', 'ReturnWorkspaceOutputs', 'on');
    
    % Robust data extraction handling variations in Simulink output object structures
    if isobject(simOut.yout)
        combined_data = simOut.yout{1}.Values.Data;
    else
        combined_data = simOut.yout;
    end
    
    % Calculate combined scalar tracking error (Objective Function)
    ep_data = combined_data(:, 1);
    epsi_data = combined_data(:, 2);
    e_train(i) = mean(ep_data) + alpha_w * mean(epsi_data);
end

%% 3. Heteroscedastic Bayesian Optimization (HBO) Loop
total_iters = num_iters_phase1 + num_iters_phase2;

for iter = 1:total_iters
    % Fit the Objective Gaussian Process (Predicts mean tracking error)
    % Uses Automatic Relevance Determination (ARD) to weigh parameter importance
    mdl_obj = fitrgp(X_train, e_train, 'KernelFunction', 'ardsquaredexponential', 'PredictMethod', 'exact');
    
    % Calculate the absolute prediction residuals from the objective GP
    e_pred = predict(mdl_obj, X_train);
    residuals = abs(e_train - e_pred);
    
    % Fit the Noise Gaussian Process (Models heteroscedasticity/variance)
    % Identifies unstable regions in the parameter space that cause erratic flight
    mdl_noise = fitrgp(X_train, residuals, 'KernelFunction', 'ardsquaredexponential', 'PredictMethod', 'exact');
    
    current_best = min(e_train);
    
    % Define the custom acquisition function wrapper
    acq_fun = @(x) hbo_acquisition(x, mdl_obj, mdl_noise, current_best);
    
    % Optimize the acquisition function using a global surrogate solver
    % PlotFcn is disabled to prevent background render lag during the physics loop
    options = optimoptions('surrogateopt', 'MaxFunctionEvaluations', 50, 'Display', 'off', 'PlotFcn', []);
    next_X = surrogateopt(acq_fun, bounds(:,1)', bounds(:,2)', options);
    
    % Inject the newly optimized candidate parameters into the workspace
    assignin('base', 'Kp_pos', next_X(1:3)');
    assignin('base', 'Kd_pos', next_X(4:6)');
    assignin('base', 'Kp_ang', next_X(7:9)');
    assignin('base', 'Kd_ang', next_X(10:12)');
    
    % Execute simulation with the candidate parameters
    simOut = sim('unmanned_aerial_vehicle_model.slx', 'ReturnWorkspaceOutputs', 'on');
    
    if isobject(simOut.yout)
        combined_data = simOut.yout{1}.Values.Data;
    else
        combined_data = simOut.yout;
    end
    
    % Evaluate and append the true error to the training dataset
    ep_data = combined_data(:, 1);
    epsi_data = combined_data(:, 2);
    next_e = mean(ep_data) + alpha_w * mean(epsi_data);
    
    X_train = [X_train; next_X];
    e_train = [e_train; next_e];
    fprintf('Iteration %d | Error: %.4f\n', iter, next_e);
end

%% 4. Post-Optimization Assignment
% Identify and extract the global minimum error and corresponding parameters
[best_e, min_idx] = min(e_train);
best_params = X_train(min_idx, :);

% Load the ultimate optimal PD gains into the workspace for final plotting/validation
assignin('base', 'Kp_pos', best_params(1:3)');
assignin('base', 'Kd_pos', best_params(4:6)');
assignin('base', 'Kp_ang', best_params(7:9)');
assignin('base', 'Kd_ang', best_params(10:12)');
fprintf('Optimization Complete. Ultimate Best Error: %.4f\n', best_e);

warning('on', 'all');

%% 5. Custom Acquisition Function
function ei = hbo_acquisition(x, mdl_obj, mdl_noise, current_best)
    % Predict expected mean error and model uncertainty
    [mu, sig_obj] = predict(mdl_obj, x);
    
    % Predict flight instability (heteroscedastic noise)
    sig_noise = predict(mdl_noise, x);
    
    % Calculate total variance (Objective Uncertainty + Physical Noise)
    total_sig = sqrt(sig_obj.^2 + sig_noise.^2);
    
    % Compute Expected Improvement (EI) using standard normal CDF/PDF
    % Penalizes candidates with high noise variance even if predicted mean error is low
    Z = (current_best - mu) ./ (total_sig + 1e-9);
    ei = -((current_best - mu) .* normcdf(Z) + total_sig .* normpdf(Z));
end