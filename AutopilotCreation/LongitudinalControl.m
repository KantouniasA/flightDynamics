%% Longitudinal autopilot PiperM500 variable generation

%% Generate plant
InitialConditions   = deg2rad([ 0;  
                                0;  
                                1;  
                                0]);

% Generate the state - system matrix
ALong           = [	-0.0141295      0.0580298       0           -9.81
                    -0.153916    	-3.68238     	121.907   	0
                    -0.000705621 	-1.19305      	-15.809 	0
                 	0             	0            	1        	0];

ALat            = [	-0.256356    	-0.897214    	-125.955 	9.81
                    -0.765269    	-24.402      	2.53297  	0
                    0.45176         -1.85712     	-3.1953  	0
                  	0            	1           	0         	0];            
                
ELong            = [1,  0,  0,  0;
                    0,  1,  0,  0
                    0,  0,  1,  0
                    0,  0,  0,  1];
              
% Generate the input - control matrices
BLong           =   [	-0.1703679
                        -50.18734
                        -182.9451
                    	0];

% Generate the output matrix
CLong                   =   [1,     0,  0,  0;
                             0,     1,  0,  0
                             0,     0,  1,  0
                             0,     0,  0,  1];
                         
% Generate the feedforward matrix
DLong                   =   [0; 
                             0
                             0
                             0];
                 
%% Actuator modeling
% Assume elevators hydraulic actuator
actuatorGain            = 1;
actuatorTimeConstant    = 5; 
actuatorNum             = actuatorGain*actuatorTimeConstant;
actuatorDen             = [1,   actuatorTimeConstant];
actuatorSys             = tf(actuatorNum,actuatorDen);

%% Gyroscope modeling
gyroscopeRateGain       = 1;

%% Calculate wich response is considered satisfying (Base of thumbprint rule)
omegaNGood              = 3;
zetaGood                = 0.6;

p1Good                  = -zetaGood*omegaNGood + 1i*omegaNGood*sqrt(1-zetaGood^2);
p2Good                  = -zetaGood*omegaNGood - 1i*omegaNGood*sqrt(1-zetaGood^2);

shortPeriodSysDenGood   = poly([p1Good, p2Good]);
