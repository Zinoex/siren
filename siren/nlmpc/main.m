Duration = 20;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
for ct = 1:(20/Ts)
    % Set references
    if ct*Ts<10
        yref = yref1;
    else
        yref = yref2;
    end
    % Correct previous prediction using current measurement 
    xk = correct(EKF, y);
    % Compute optimal control moves 
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
    % Predict prediction model states for the next iteration
    predict(EKF, [mv; Ts]);
    % Implement first optimal control move and update plant states.
    x = pendulumDT0(x,mv,Ts);
    % Generate sensor data with some white noise
    y = x([1 3]) + randn(2,1)*0.01; 
    % Save plant states for display.
    xHistory = [xHistory x]; %#ok<*AGROW>
    waitbar(ct*Ts/20,hbar);
end
close(hbar);