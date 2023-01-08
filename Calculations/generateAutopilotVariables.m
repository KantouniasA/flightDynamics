%% Longitudinal autopilot PiperM500 variable generation (Pitch rate dumper)
% This script load generates and save all the varialbles needed
% for autopilot pitch generation (short period aproximation)

%% Generate plant
% Generate the state - system matrix
% (From _________State matrices_________ of XFLR.log file) 
AShortPeriod     =  [-1.98701,   66.6537;
                    -0.55834,   -8.45195];
                
% Generate the input - control matrices
% (From _________Control matrices_________ of XFLR.log file)
BShortPeriod    =   [-139.3498;
                     -476.9312];

% Generate the output matrix
% Consider as output the angular velocity q
CShortPeriod    =   [0,     1];

% Generate the feedforward matrix
DShortPeriod    =   [0];

% Generate the system
[shortPeriodSysNum,shortPeriodSysDen] = ss2tf(AShortPeriod,BShortPeriod,CShortPeriod,DShortPeriod);

%% Actuator modeling
% Assume elevators hydraulic actuator
actuatorGain            = 1;
actuatorTimeConstant    = 5; 
actuatorNum             = actuatorGain*actuatorTimeConstant;
actuatorDen             = [1,   actuatorTimeConstant];

%% Gyroscope modeling
gyroscopeRateGain       = 1;

%% Generate pitch dumper controler
% Desired plant poles
pole1   = -1.8 + 1i*2.4;
pole2   = -1.8 - 1i*2.4;

% PI controler design




