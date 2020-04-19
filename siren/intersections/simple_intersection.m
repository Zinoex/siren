%Simple intersection with four lanes and no left turns
num_signals = 4;

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

green_interval_matrix = ...
[
    0, 5, 0, 5; 
    5, 0, 5, 0;
    0, 5, 0, 5; 
    5, 0, 5, 0
];

yellow_time_vector = [4; 4; 4; 4];
amber_time_vector = [2; 2; 2; 2];
minimum_green_vector = [6; 6; 6; 6];

% Visualizations
tlPositions = ...
[...
    0.7, 0.625;...
    0.625, 0.3;...
    0.3, 0.375;...
    0.375, 0.7 ...
];

labelPositions = ...
[...
    0.75, 0.7;...
    0.70, 0.25;...
    0.10, 0.25;...
    0.10, 0.75 ...
];

clear segments;

segments(1).lanes = 2;
segments(1).double_yellow = 1;
segments(1).start = [-0.4 0.5];
segments(1).end = [0.4 0.5];

segments(2).lanes = 2;
segments(2).double_yellow = 1;
segments(2).start = [1.4 0.5];
segments(2).end = [0.6 0.5];

segments(3).lanes = 2;
segments(3).double_yellow = 1;
segments(3).start = [0.5 1];
segments(3).end = [0.5 0.6];

segments(4).lanes = 2;
segments(4).double_yellow = 1;
segments(4).start = [0.5 0];
segments(4).end = [0.5 0.4];

clear ped_xings;

clear connectors;

connectors(1).coords = [0.4 0.4; 0.4 0.6; 0.6 0.6; 0.6 0.4];
connectors(1).sides = false;