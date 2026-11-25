# MATLAB and Simulink model

The repository also contains a quadrotor tracking model with cascaded PD control and Bayesian gain tuning. Use MATLAB with Simulink, Statistics and Machine Learning Toolbox, and the optimisation toolboxes required by the scripts.

Run [run_optimisation.m](../run_optimisation.m) from the repository root to tune the model, then [plot_results.m](../plot_results.m) to generate tracking and control-effort plots. Open [unmanned_aerial_vehicle_model.slx](../unmanned_aerial_vehicle_model.slx) to inspect the plant and controller. Sample output and the model overview are in [figures/](../figures/).

The [Arduino and Python setup](setup.md) runs independently of MATLAB.
