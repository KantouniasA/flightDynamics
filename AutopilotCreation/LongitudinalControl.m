%% Figure options
FONTSIZE    = 15;
LINEWIDTH   = 2;
POSITION    = [50, 50, 1100, 700];

%% Longitudinal autopilot PiperM500 variable generation (Control based on pitch rate)

%% Generate plant
% Generate initial conditions
InitialConditions   =     	   [0;  
                                0;  
                                0;  
                                0];

% Generate input step command                            
stepCommand         =           1;                            

% Generate the state - system matrix
ALong           = [	-0.0141295      0.0580298       0           -9.81
                    -0.153916    	-3.68238     	121.907   	0
                    -0.000705621 	-1.19305      	-15.809 	0
                 	0             	0            	1        	0];
                
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

% Generate short period system for pitch damper control generation
AShortPeriod            = ALong(2:3,2:3);
BShortPeriod            = BLong(2:3);
CShortPeriod            = CLong(2:3,2:3);
DShortPeriod            = DLong(2:3);

% Generate open loop short period tranfer funcion
[shortPeriodSysNum,shortPeriodSysDen]   = ss2tf(AShortPeriod,BShortPeriod,CShortPeriod,DShortPeriod);
shortPeriodSys                          = tf(shortPeriodSysNum(2,:),shortPeriodSysDen);
              
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
Td =  0.1;
derivativeSystem = tf([Td, 1],[0, 1]);

%% Calculate wich response is considered satisfying (Base of thumbprint rule)
% From finger-print diagram we obtain:
omegaNGood              = [3, 3.7, 3.7, 3.6, 3.3, 2.8, 2.5, 2.3];
zetaGood                = [0.4, 0.6, 0.8, 1.0, 1.3, 1.0, 0.8, 0.6];

p1Good = zeros(1,length(omegaNGood));
p2Good = zeros(1,length(omegaNGood));

% Transfer the satisfaction space at the imaginary plane
for iResponce = 1:length(omegaNGood)
    p                       = roots([1,  2*omegaNGood(iResponce)*zetaGood(iResponce), omegaNGood(iResponce)^2]);
    p1Good(iResponce)    	= p(1);
    p2Good(iResponce)    	= p(2);
end

%% Generate root locus for short period system
figurePD = figure;
figurePD.Position = POSITION;

rlocus(-derivativeSystem*actuatorSys*shortPeriodSys)

hold on
c = [0.8 0.7 0.8];
fingerPrint   = fill([real(p1Good);real(p2Good)]',[imag(p1Good);imag(p2Good)]',c);
fingerPrint(1).LineWidth = 1;
fingerPrint(1).EdgeColor = [0.7,0.6,0.7];
fingerPrint(2).LineWidth = 1;
fingerPrint(2).EdgeColor = [0.7,0.6,0.7];
hold off

grid on
axis equal
title('Root locus map for the pitch rate controller gain','fontsize',FONTSIZE)
legend({'Closed loop system poles','Satisfication space'},'FontSize',FONTSIZE)
xlabel('σ','fontsize',FONTSIZE)
ylabel('ω_d','fontsize',FONTSIZE)

%% Calculate P and D gains based on root locus diagram
PGain   = 0.211;
DGain   = PGain*Td;

%% Calculate closed loop pitch damper system 
closedLoopSystem    = feedback(PGain*derivativeSystem*actuatorSys*shortPeriodSys,-gyroscopeSystem);

%% Insert integrative control to the system
Integrator              = tf(-1,[1,0]);
openLoopSystemIngrated	= closedLoopSystem*Integrator;

%% Generate root locus for short period system with integrative control
figureI = figure;
figureI.Position = POSITION;
rlocus(openLoopSystemIngrated)

hold on
fingerPrint   = fill([real(p1Good);real(p2Good)]',[imag(p1Good);imag(p2Good)]',c);
fingerPrint(1).LineWidth = 1;
fingerPrint(1).EdgeColor = [0.7,0.6,0.7];
fingerPrint(2).LineWidth = 1;
fingerPrint(2).EdgeColor = [0.7,0.6,0.7];
hold off

grid on
axis equal
title('Root locus map for the integrative gyroscope gain','fontsize',FONTSIZE)
legend({'Closed loop system poles','Satisfication space'},'FontSize',FONTSIZE)
xlabel('σ','fontsize',FONTSIZE)
ylabel('ω_d','fontsize',FONTSIZE)

%% Calculate I gain 
IGain = 17.4;
