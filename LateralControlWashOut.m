%% Longitudinal autopilot PiperM500 variable generation 

%% Generate plant
% Generate initial conditions
InitialConditions   =     	   [0;  
                                0;
                                1;
                                0];

% Generate input step command                            
stepCommand         =           0;                            

% Generate the state - system matrix
ALat            = [	-0.256356    	-0.897214    	-125.955 	9.81
                    -0.765269    	-24.402      	2.53297  	0
                    0.45176         -1.85712     	-3.1953  	0
                  	0            	1           	0         	0];  

ELat             = [1,  0,  0,  0;
                    0,  1,  0,  0
                    0,  0,  1,  0
                    0,  0,  0,  1];
              
% Generate the input - control matrices
BLat            =   [	14.7975
                        8.7650
                        -37.5943
                    	0       ];

% Generate the output matrix
CLat                    =   [1,     0,  0,  0;
                             0,     1,  0,  0
                             0,     0,  1,  0
                             0,     0,  0,  1];
                         
% Generate the feedforward matrix
DLat                    =   [0; 
                             0
                             0
                             0];

ELatWashOut                 = ELat([1,3,4],[1,3,4]);
ALatWashOut                 = ALat([1,3,4],[1,3,4]);
BLatWashOut                 = [BLat([1,3,4]),ALat([1,3,4],2)];
CLatWashOut                 = CLat([1,3,4],[1,3,4]);
DLatWashOut                 = [DLat([1,3,4]),[0;0;0]];
InitialConditionsWashOut    = InitialConditions([1,3,4]);

% Generate open loop lateral dynamics transfer function
% [LatSysNum,LatSysDen]   = ss2tf(ALat, BLat, CLat, DLat);
% LatSys                  = tf(LatSysNum(3,:),LatSysDen);
              
%% Create the actuator model
% Assume elevators hydraulic actuator
actuatorGain            = 1;
actuatorTimeConstant    = 5; 
actuatorNum             = actuatorGain*actuatorTimeConstant;
actuatorDen             = [1,   actuatorTimeConstant];
actuatorSys             = tf(actuatorNum,actuatorDen);

%% Gyroscope modeling
% Gyroscope sensor can be modeled as a simple gain
gyroscopeRateGain       = 1;
gyroscopeSystem         = tf(gyroscopeRateGain,1);

%% Pitch rate controller transfer function
% Gain is calculated after root locus study
Td                      =  0.1;

%% Calculate P and D gains based on root locus diagram
PGain   = 1.56;
DGain   = PGain*Td;

FWashOut    = 1;
FRoll       = 0.2;