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

%Starting lane states (colors)
green = [0; 0; 0; 0; 0; 0; 0; 0];
yellow = [0; 0; 0; 0; 0; 0; 0; 0];
red = [1; 1; 1; 1; 1; 1; 1; 1];
amber = [0; 0; 0; 0; 0; 0; 0; 0];

%Conflict matrix for all 8 signals
conflict_matrix = ...
[
    0, 0, 0, 0, 0, 0, 1, 1;
    0, 0, 1, 1, 1, 1, 1, 1;
    0, 1, 0, 0, 0, 0, 1, 1;
    0, 1, 0, 0, 1, 1, 0, 1;
    0, 1, 0, 1, 0, 0, 1, 1;
    0, 1, 0, 1, 0, 0, 1, 1;
    1, 1, 1, 0, 1, 1, 0, 0;
    1, 1, 1, 1, 1, 1, 0, 0
];

green_interval_matrix = ...
[
    0, 0, 0, 0, 0, 0, 6, 6;
    0, 0, 6, 6, 6, 6, 8, 8;
    0, 6, 0, 0, 0, 0, 6, 6;
    0, 6, 0, 0, 6, 6, 0, 6;
    0, 6, 0, 6, 0, 0, 8, 8;
    0, 6, 0, 6, 0, 0, 6, 6;
    7, 7, 6, 0, 6, 5, 0, 0;
    6, 6, 8, 8, 8, 8, 0, 0
];

yellow_time_vector = [4; 4; 4; 0; 4; 0; 4; 4];
amber_time_vector = [2; 2; 2; 0; 2; 0; 2; 2];
minimum_green_vector = [6; 6; 6; 6; 8; 10; 6; 6];