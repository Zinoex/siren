%A real intersection that's currently running the ATS control system
%https://www.google.com/maps/place/Eg%C3%A5+Havvej+%26+Gren%C3%A5vej,+8250+Eg%C3%A5,+Denmark/@56.2126194,10.2764868,17z/data=!3m1!4b1!4m5!3m4!1s0x464c3e9753942ac9:0xbe8be68c98e63d5!8m2!3d56.2126194!4d10.2786755

% Signal 1 = A1
% Signal 2 = A1Left
% Signal 3 = A2
% Signal 4 = A2Right (arrow)
% Signal 5 = A2Bicycle
% Signal 6 = APedestrian (crossing B)
% Signal 7 = B
% Signal 8 = BBicycle (from across B)

num_signals = 8;
num_queues = 8;

S = eye(num_signals);
S(4, 3) = 1;

%Starting lane states (colors)
green = [0; 0; 0; 0; 0; 0; 0; 0];
yellow = [0; 0; 0; 0; 0; 0; 0; 0];
red = [1; 1; 1; 1; 1; 1; 1; 1];
amber = [0; 0; 0; 0; 0; 0; 0; 0];

%Conflict matrix for all 8 signals
conflict_matrix = ...
[...
    0, 0, 0, 0, 0, 0, 1, 1;...
    0, 0, 1, 1, 1, 1, 1, 1;...
    0, 1, 0, 0, 0, 0, 1, 1;...
    0, 1, 0, 0, 1, 1, 0, 1;...
    0, 1, 0, 1, 0, 0, 1, 1;...
    0, 1, 0, 1, 0, 0, 1, 1;...
    1, 1, 1, 0, 1, 1, 0, 0;...
    1, 1, 1, 1, 1, 1, 0, 0 ...
];

green_interval_matrix = ...
[...
    0, 0, 0, 0, 0, 0, 6, 6;...
    0, 0, 6, 6, 6, 6, 8, 8;...
    0, 6, 0, 0, 0, 0, 6, 6;...
    0, 6, 0, 0, 6, 6, 0, 6;...
    0, 6, 0, 6, 0, 0, 8, 8;...
    0, 6, 0, 6, 0, 0, 6, 6;...
    7, 7, 6, 0, 6, 5, 0, 0;...
    6, 6, 8, 8, 8, 8, 0, 0 ...
];

yellow_time_vector = [4; 4; 4; 0; 4; 0; 4; 4];
amber_time_vector = [2; 2; 2; 0; 2; 0; 2; 2];
minimum_green_vector = [6; 6; 6; 4; 8; 10; 6; 6];

% Visualizations
tlPositions = ...
[...
    0.7, 0.65;...
    0.7, 0.5;...
    0.3, 0.45;...
    0.3, 0.3;...
    0.3, 0.225;...
    0.4, 0.175;...
    0.55, 0.225;...
    0.5, 0.775 ...
];

labelPositions = ...
[...
    0.75, 0.7;...
    0.8, 0.5;...
    0.05, 0.5;...
    0.05, 0.3; ...
    0.05, 0.165;...
    0.42, 0.11;...
    0.5, 0.3;...
    0.575, 0.8 ...
];

clear segments;

% A2 far
segments(1).lanes = 4;
segments(1).double_yellow = 2;
segments(1).start = [-0.4 0.5];
segments(1).end = [-0.2 0.5];

% A2 close
segments(2).lanes = 5;
segments(2).double_yellow = 2;
segments(2).start = [0 0.5];
segments(2).end = [0.35 0.5];

% A1 far
segments(3).lanes = 4;
segments(3).double_yellow = 2;
segments(3).start = [1.4 0.5];
segments(3).end = [1.2 0.5];

% A1 close
segments(4).lanes = 5;
segments(4).double_yellow = 2;
segments(4).start = [1 0.5];
segments(4).end = [0.65 0.5];

% BBicycle
segments(5).lanes = 1;
segments(5).width = 0.05;
segments(5).start = [0.5 1];
segments(5).end = [0.5 0.75];

% B
segments(6).lanes = 3;
segments(6).double_yellow = 1;
segments(6).start = [0.5 0];
segments(6).end = [0.5 0.25];

% A2 bicycle
segments(7).lanes = 1;
segments(7).width = 0.03;
segments(7).start = [0 0.225];
segments(7).end = [0.35 0.225];

% TODO: Arrows

clear ped_xings;

ped_xings(1).start = [0.37 0.175];
ped_xings(1).end = [0.63 0.175];

clear connectors;

% A2
connectors(1).coords = [-0.2 0.7; 0 0.75; 0 0.25; -0.2 0.3];
connectors(1).sides = true;
connectors(1).double_yellow = [-0.2 0.5; 0 0.55];

% A1
connectors(2).coords = [1 0.75; 1.2 0.7; 1.2 0.3; 1 0.25];
connectors(2).sides = true;
connectors(2).double_yellow = [1.2 0.5; 1 0.45];

% Center
connectors(3).coords = [0.35 0.25; 0.35 0.75; 0.65 0.75; 0.65 0.25];
connectors(3).sides = false;