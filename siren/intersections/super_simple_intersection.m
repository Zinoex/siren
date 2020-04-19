% Super simple intersection with two lanes. 
% Could be a one-lane bidirectional light controlled street.
num_signals = 2;

%Starting lane states (colors)
green = [1; 0];
yellow = [0; 0];
red = [0; 1];
amber = [0; 0];

%Conflict matrix for all 2 lanes (no turns in this model)
conflict_matrix = ...
[...
    0, 1;...
    1, 0 ...
];

green_interval_matrix = ...
[...
    0, 15;...
    15, 0 ...
];

yellow_time_vector = [4; 4];
amber_time_vector = [2; 2];
minimum_green_vector = [6; 6];

% Visualizations
tlPositions = ...
[...
    0.75, 0.65;...
    0.25, 0.35 ...
];

labelPositions = ...
[...
    0.55, 0.7;...
    0.25, 0.25 ...
];

clear segments;

segments(1).lanes = 2;
segments(1).start = [-0.4 0.5];
segments(1).end = [0.25 0.5];

segments(2).lanes = 1;
segments(2).start = [0.35 0.5];
segments(2).end = [0.65 0.5];

segments(3).lanes = 2;
segments(3).start = [0.75 0.5];
segments(3).end = [1.4 0.5];

clear ped_xings;

clear connectors;

connectors(1).coords = [0.25 0.6; 0.35 0.55; 0.35 0.45; 0.25 0.4];
connectors(1).sides = true;

connectors(2).coords = [0.75 0.4; 0.65 0.45; 0.65 0.55; 0.75 0.6];
connectors(2).sides = true;