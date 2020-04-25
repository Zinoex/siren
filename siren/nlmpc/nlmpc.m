% Create non-linear MPC
nx = 8 * num_signals;
ny = 3 * num_signals;
nu = 4 * num_signals;
nlobj = nlmpc(nx, ny, nu);

% Configure MPC
nlobj.Ts = 1;
nlobj.PredictionHorizon = 40;
nlobj.ControlHorizon = 30;

nlobj.Model.StateFcn = "StateFn";
nlobj.Model.IsContinuousTime = false;

nlobj.Optimization.CustomEqConFcn = "ConstraintFn";