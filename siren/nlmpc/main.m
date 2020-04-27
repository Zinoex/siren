function main(intersection_name)
    if nargin == 0
        intersection_name = "super_simple_intersection.m";
    end
    run(intersection_name)
    
    Ts = 1;
    nx = 8 * num_signals;
    ny = 3 * num_signals;
    mvIndex = 1:4 * num_signals;
    mdIndex = 4 * num_signals + 1:num_signals * 6;
    
    nlobj = nlmpc(nx, ny, 'MV', mvIndex, 'MD', mdIndex);
   
    % Configure MPC
    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = 10;
    nlobj.ControlHorizon = 5;
    nlobj.Model.NumberOfParameters = 7;
    
    nlobj.Model.StateFcn = "StateFn";
    nlobj.Model.IsContinuousTime = false;
    
    nlobj.Model.OutputFcn = "OutputFn";
    nlobj.Jacobian.OutputFcn = "OutputJacobian";

%     nlobj.Optimization.ReplaceStandardCost = false;
    nlobj.Optimization.UseSuboptimalSolution = true;
    
    for i = mvIndex
        nlobj.ManipulatedVariables(i).Min = 0;
        nlobj.ManipulatedVariables(i).Max = 1;
    end
    nlobj.Optimization.CustomEqConFcn = "ConstraintFn";
    nlobj.Optimization.CustomIneqConFcn = "IneqConstraintFn";

    % Print parameters after initialization
    nlobj

    % Construct initial values
    xk = zeros(nx, 1);
    mv = [0 0 1 1 0 0 0 0];
    md = ones(1, 2 * num_signals);

%     md(num_signals + 1:2 * num_signals) = md(num_signals + 1:2 * num_signals) * 5
    
    % Construct static parameters
    yref = zeros(1, 3 * num_signals);
    nloptions = nlmpcmoveopt;
    nloptions.Parameters = {Ts, conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, minimum_green_vector, num_signals};

    save("nlmpc_model.mat", "nlobj")
    for i = 1:10
        [mv, nloptions, info] = nlmpcmove(nlobj, xk, mv, yref, md, nloptions);
        move_flag = info.ExitFlag;
        if move_flag < 0
            fprintf("Error on iteration: %i, no feasible solution found. Flag is negative: %i.\n", i, move_flag);
            input("Press any key to exit.")
            return
        end
        uk = [mv; md'];
        xk = StateFn(xk, uk, ...
            Ts, conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, minimum_green_vector, num_signals);
        
        q = xk(5 * num_signals + 1:6 * num_signals)
        lights_current = mv(1:4 * num_signals)

    end
end
%     for ct = 1:20
%         [mv, nloptions, info] = nlmpcmove(nlobj, xk, mv, yref, md, nloptions)
%     % hbar = waitbar(0,'Simulation Progress');
%     xHistory = x;
%     for ct = 1:(20/Ts)
%         % Set references
% 
%         % Correct previous prediction using current measurement 
%         xk = correct(EKF, y);
%         % Compute optimal control moves 
%         [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
%         % Predict prediction model states for the next iteration
%         predict(EKF, [mv; Ts]);
%         % Implement first optimal control move and update plant states.
%         x = pendulumDT0(x,mv,Ts);
%         % Generate sensor data with some white noise
%         y = x([1 3]) + randn(2,1)*0.01; 
%         % Save plant states for display.
%         xHistory = [xHistory x]; %#ok<*AGROW>
%         waitbar(ct*Ts/20,hbar);
%     end
% close(hbar);