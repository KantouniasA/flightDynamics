%% Longitudinal autopilot PiperM500 variable generation (Pitch rate dumper)
% This script load generates and save all the varialbles needed
% for autopilot pitch generation (short period aproximation)

%% Generate plant
% Generate the state - system matrix
% (From _________State matrices_________ of XFLR.log file) 
EShordPeriod     = [1,  0;
                    0,  1];

AShortPeriod     =  [-1.98701,   66.6537;
                    -0.55834,   -8.45195];
                
% Generate the input - control matrices
% (From _________Control matrices_________ of XFLR.log file)
BShortPeriod    =   [-139.3498;
                     -476.9312];

% Generate the output matrix
% Consider as output the angular velocity q
CShortPeriod            =   [1,     0;
                             0,     1];
                         
CShortPeriodControl     =   [0,     1];

% Generate the feedforward matrix
DShortPeriod            =   [0; 
                             0];
                 
DShortPeriodControl     =   0;                

% Generate the system
[shortPeriodSysNum,shortPeriodSysDen] = ss2tf(AShortPeriod,BShortPeriod,CShortPeriodControl,DShortPeriodControl);
kq  = shortPeriodSysNum(2);
T82 = kq/shortPeriodSysNum(3);
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
pole1   = -2;%-1.8 + 1i*2.4;
pole2   = -100;%-1.8 - 1i*2.4;
pole3   = -200;

% Desired characteristic equation
characteristEquationDesired = poly([pole1,pole2,pole3]);

% PID controler design
syms KD KP KI
designEq1 = KP/KD + 1/T82 + 1/(KD*kq) ==  characteristEquationDesired(2);
designEq2 = KI/KD + KP/(KD*T82) + shortPeriodSysDen(2)/(KD*kq) == characteristEquationDesired(3);
designEq3 = KI/KD/T82 + shortPeriodSysDen(3)/(KD*kq) == characteristEquationDesired(4);

[KDD, KPP, KII] = solve([designEq1,designEq2,designEq3],[KD,KP,KI]);
KDD = double(KDD);
KPP = double(KPP);
KII = double(KII);

simOut = sim('SortPeriodSimulation.slx');

%% Figures
figure
plot(simOut.W.Time,simOut.W.Data)

figure
plot(simOut.Q.Time,simOut.Q.Data)
