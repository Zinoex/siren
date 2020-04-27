function cineq = IneqConstraintFn(X, U, e, data, Ts, conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, minimum_green_vector, num_signals)
    %% INPUTS
    % X - State trajectory from time k to k+p
    %     matrix of size (p+1) x N
    % U - Input trajector from time k to k+p
    %     matrix of size (p+1) x N
    % e - Slack variable for constraint softening 
    %     positive scalar
    
%     X = X(2:end, :);
%     U = U(2:end, :);
%     [time_steps, ~] = size(X);
%     signals = U(:, 1:4 * num_signals);
%     green = reshape(signals(:, index(0, num_signals)), [], 1);
%     red = reshape(signals(:, index(1, num_signals)), [], 1);
%     yellow = reshape(signals(:, index(2, num_signals)), [], 1);
%     amber = reshape(signals(:, index(3, num_signals)), [], 1);
%    
% 
%     % Times has repeated data of the form:
%     % tg1, tg2, tr1, tr2, ...
%     times = X(:, 1:4 * num_signals);
%     green_time = reshape(times(:, index(0, num_signals):index(0, num_signals)+num_signals-1), [], 1);
%     red_time = reshape(times(:, index(1, num_signals):index(1, num_signals)+num_signals-1), [], 1);
%     yellow_time = reshape(times(:, index(2, num_signals):index(2, num_signals)+num_signals-1), [], 1);
%     amber_time = reshape(times(:, index(3, num_signals):index(3, num_signals)+num_signals-1), [], 1);
% 
%     yellow_max_time_future = reshape(repmat(yellow_time_vector', time_steps, 1), [], 1);
%     amber_max_time_future = reshape(repmat(amber_time_vector', time_steps, 1), [], 1);
%     red_max_time_future = reshape(repmat(yellow_time_vector', time_steps, 1), [], 1);
% 
%   
%     amber_max_time_constraint = amber_time - amber_max_time_future .* amber;
%     yellow_max_time_constraint = yellow_time - yellow_max_time_future .* yellow;
%     red_max_time_constraint = red_time - red_max_time_future .* red;
%     cineq = [red_max_time_constraint'];
%     cineq = [yellow_max_time_constraint; amber_max_time_constraint];
%     max_amber_time_stack = repmat(amber_time_vector, time_steps, 1);
%     max_yellow_time_stack = repmat(yellow_time_vector, time_steps, 1);
%     min_green_time_stack = repmat(minimum_green_vector, time_steps, 1);
% 
%     
%     max_amber_constraint = amber_time - max_amber_time_stack .* green;
%     max_yellow_constraint = yellow_time - max_yellow_time_stack .* yellow;
%     
%     fake_red_constraint = indicator(red_time - max_yellow_time_stack .* red);
%     cineq = fake_red_constraint;
%     cineq = max_amber_constraint;
%     [green_time, green_size] = mstack(green_time);
%     [red_time, red_size] = mstack(red_time);
%     [yellow_time, yellow_size] = mstack(yellow_time);
%     [amber_time, amber_size] = mstack(amber_time)
%     
% %     min_green_constraint = minimum_green_vector .* yellow - green_time;
% %     min_green_vec = reshape(min_green_constraint, [], 1);
% %     cineq = min_green_vec;
%     max_amber_constraint = reshape(amber_time_vector_rep, [], 1) .*green - maxim
%     max_amber_vec = reshape(max_amber_constraint, [], 1)
%     cineq = [max_amber_vec]
end    

function [arr_stack, stack_size] = mstack(m)
    arr_stack = reshape(m, [], 1);
    stack_size = size(arr_stack, 1);
end

function idx = index(i, num_signals)
    idx = i * num_signals + 1:(i + 1) * num_signals;
end
    
function y = indicator(x)
    y = x >= 1;
end

function [ineq1, ineq2, ineq3] = logical_product(aux, l1, l2)
    ineq1 = - l1 + aux;
    ineq2 = - l2 + aux;
    ineq3 = l1 + l2 - aux - 1;
end