%Simple intersection with four lanes and no left turns

%Starting lane states (colors)
green = [1; 0; 1; 0];
yellow = [0; 0; 0; 0];
red = [0; 1; 0; 1];
amber = [0; 0; 0; 0];

%Conflict matrix for all 4 lanes (no turns in this model)
conflict_matrix = ...
[
    0, 1, 0, 1; 
    1, 0, 1, 0;
    0, 1, 0, 1; 
    1, 0, 1, 0
];


green_interval_matrix = [[0 5] [5 0]];  %LOOK HERE: Should be size 4x4
yellow_time_vector = [3 3];  % LOOK HERE: Should be length 4
amber_time_vector = [3 3];   % LOOK HERE: Should be length 4