function ceq = ConstraintFn(X, U, data)

% Sum of lights = 1
signals = U(:, 1:4 * num_signals);
green = signals(:, index(0));
red = signals(:, index(1));
yellow = signals(:, index(2));
amber = signals(:, index(3));

sum_of_lights = green + red + yellow + amber - 1;
stacked_sum_of_lights = reshape(sum_of_lights, [], 1);
size_lights = size(stacked_sum_of_lights, 1);

ceq(1:size_lights) = stacked_sum_of_lights;

% Pairwise hadamard
gr = green .* red;
gy = green .* yellow;
ga = green .* amber;
ry = red .* yellow;
ra = red .* amber;
ya = yellow .* amber;

stacked_pairwise = reshape([gr; gy; ga; ry; ra; ya], [], 1);
size_pairwise = size(stacked_pairwise, 1);

ceq(size_lights + 1: size_lights + size_pairwise) = stacked_pairwise;

% Transition (e.g. from red to amber is allowed) constraints


% Conflict
p = data.PredictionHorizon;

function idx = index(i)
idx = i * num_signals + 1:(i + 1) * num_signals;