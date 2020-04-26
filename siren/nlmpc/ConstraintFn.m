function ceq = ConstraintFn(X, U, data, ...
    conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, minimum_green_vector, num_signals)

    % Sum of lights = 1
    signals = U(:, 1:4 * num_signals);
    green = signals(:, index(0, num_signals));
    red = signals(:, index(1, num_signals));
    yellow = signals(:, index(2, num_signals));
    amber = signals(:, index(3, num_signals));

    sum_of_lights = green + red + yellow + amber - 1;

    [stacked_lights, size_lights] = mstack(sum_of_lights);
    ceq(1:size_lights) = stacked_lights;

    % Pairwise hadamard
    gr = green .* red;
    gy = green .* yellow;
    ga = green .* amber;
    ry = red .* yellow;
    ra = red .* amber;
    ya = yellow .* amber;

    [stacked_pairwise, size_pairwise] = mstack([gr; gy; ga; ry; ra; ya]);
    ceq((1:size_pairwise) + size_lights) = stacked_pairwise;

    % Transition (e.g. from red to amber is allowed) constraints
%     p = data.PredictionHorizon;
% 
%     old_idx = 1:p - 1;
%     new_idx = 2:p;
% 
%     yt = yellow(old_idx, :) .* (green(new_idx, :) + amber(new_idx, :));
%     gt = green(old_idx, :) .* (red(new_idx, :) + amber(new_idx, :));
%     at = amber(old_idx, :) .* (red(new_idx, :) + yellow(new_idx, :));
%     ryt = red(old_idx, :) .* yellow(new_idx, :);
%     rgt = red(old_idx, :) .* green(new_idx, :) .* repmat(amber_time_vector, 1, p - 1).';
%     rat = red(old_idx, :) .* amber(new_idx, :) .* (indicator(repmat(amber_time_vector, 1, p - 1).') - 1);
% 
%     [stacked_trans, size_trans] = mstack([yt; gt; at; ryt; rgt; rat]);
%     ceq((1:size_trans) + size_lights + size_pairwise) = stacked_trans;
% 
%     % Conflict
%     non_blocking = green + yellow + amber;

%     for i = 1:size(non_blocking, 2)
%         ceq(i + size_trans + size_lights + size_pairwise) = non_blocking(i, :) * conflict_matrix * non_blocking(i, :).';
%     end
    ceq = ceq';
function idx = index(i, num_signals)
    idx = i * num_signals + 1:(i + 1) * num_signals;

function [arr_stack, stack_size] = mstack(m)
    arr_stack = reshape(m, [], 1);
    stack_size = size(arr_stack, 1);

function y = indicator(x)
    y = x >= 1;