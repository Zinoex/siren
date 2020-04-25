function main(intersection_name)
    if nargin == 0
        intersection_name = "super_simple_intersection.m"
    end
    run(intersection_name)
    Ts = 20;
    nx = 8 * num_signals;
    ny = 3 * num_signals;
    nu = 6 * num_signals;
%     nlobj = nlmpc(nx, ny, nu);
    mvIndex = (1:num_signals *5);
    mdIndex = (num_signals * 5 + 1:num_signals * 7);
    
    nlobj = nlmpc(nx,ny,'MV',mvIndex,'MD',mdIndex)

    % Configure MPC
    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = 40;
    nlobj.ControlHorizon = 30;
    
    nlobj.Model.StateFcn = "StateFn";
    nlobj.Model.IsContinuousTime = false;
    nlobj.Model.OutputFcn = "OutputFn"
%     nlobj.Optimization.CustomEqConFcn = "ConstraintFn";

    xk = zeros(nx, 1);
    mv = zeros(5 * num_signals, 1);
    md = zeros(2 * num_signals);
    yref = zeros(3 * num_signals, 1);
    nloptions = nlmpcmoveopt;
    nloptions.Parameters = {Ts};
    size(mv)
    [mv, nloptions, info] = nlmpcmove(nlobj, xk, mv, yref, md, nloptions)
    Duration = 20;
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