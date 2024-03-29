%% UnderdrivernTriplePendulumMotor Parameters
PI = 3.141592653589793;
w = 2*PI;

L1 = 0.325;
L2 = 0.2;
L3 = 0.2;
H = 0.05;
W = 0.05;

%% Link1 data
link1_m = 1.915;
link1_CM = [82.936*1e-3 0 0];
link1_MI = [0 0 38896.101*1e-6];
% link1_MI = [8.333333333338782e-04 0.0271 0.0271];
link1_PI = [0 0 0];

%% Link2 data
link2_m = 1.469;
link2_CM = [65.468*1e-3 0 0];
link2_MI = [0 0 13964.542*1e-6];
link2_PI = [0 0 0];

%% Link3 data
link3_m = 1.141;
link3_CM = [27.337*1e-3 0 0];
link3_MI = [0 0 5801.831*1e-6];
link3_PI = [0 0 0 ];

%% Acrobot parameters
%AL1 = 0.325;
%AL2 = 0.4;
%AL3 = 0.0;
AL1 = 0.4;
AL2 = 0.6;

%% Acrobot Link1 data
Alink1_m = 0.49; %1.915
Alink1_CM = [0 0 0]; %82.936*1e-3
%%Alink1_CM = [-0.0286 0 0]; %82.936*1e-3
Alink1_MI = [0 0 0.0036]; %0.0040008004
Alink1_PI = [0 0 0];

%% Acrobot Link2 data
Alink2_m = 0.11; %1
Alink2_CM = [0.0 0 0]; %36.232*1e-3 0.1364
Alink2_MI = [0 0 0.0043]; %32871.952*1e-63 0.0063465456
Alink2_PI = [0 0 0];

%% Acrobot Link3 data
Alink3_m = 0.0;
Alink3_CM = [0.01 0 0];
Alink3_MI = [0 0 0.0];
Alink3_PI = [0 0 0.0];

















