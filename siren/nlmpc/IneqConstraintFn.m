function cineq = function(X, U, e, data, ...
    conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, minimum_green_vector, num_signals)
    %% INPUTS
    % X - State trajectory from time k to k+p
    %     matrix of size (p+1) x N
    % U - Input trajector from time k to k+p
    %     matrix of size (p+1) x N
    % e - Slack variable for constraint softening 
    %     positive scalar

    signals = U(:, 1:4 * num_signals);
    green = signals(:, index(0));
    red = signals(:, index(1));
    yellow = signals(:, index(2));
    amber = signals(:, index(3));
    
    times = X(:, 1:4 * num_signals);
    green_time = times(:, index(0));
    red_time = times(:, index(1));
    yellow_time = times(:, index(2));
    amber_time = times(:, index(3));
    
    
    min_green_constraint = minimum_green_vector .* yellow - green_time;
    min_green_vec = reshape(min_green_constraint, [], 1);
    
%     max_amber_constraint = 
    
    cineq = min_green_vec;
    


function idx = index(i)
    idx = i * num_signals + 1:(i + 1) * num_signals;