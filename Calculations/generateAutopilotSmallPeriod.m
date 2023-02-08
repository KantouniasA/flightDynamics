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
% Consider as output the angular velocity q (Anadrasi taxititas proneusis)
CShortPeriod            =   [1,     0;
                             0,     1];
                         
CShortPeriodControl     =   [0,     1];

% Generate the feedforward matrix
DShortPeriod            =   [0; 
                             0];
                 
DShortPeriodControl     =   0;                

%% Generate the open loop system 
[shortPeriodSysNum,shortPeriodSysDen]   = ss2tf(AShortPeriod,BShortPeriod,CShortPeriodControl,DShortPeriodControl);
shortPeriodSys                          = tf(shortPeriodSysNum,shortPeriodSysDen);

shortPeriodPole                         = pole(shortPeriodSys);
shortPeriodZero                         = zero(shortPeriodSys);

% Symplyfy the plant form Gplant = kPlant*(nPlant/dPlant)
kPlant                                  = shortPeriodSysNum(2);
nPlant                                  = shortPeriodSysNum/kPlant;
dPlant                                  = shortPeriodSysDen;

% Calculate natural frequency and dumping of the system
omegaN                                  = sqrt(shortPeriodSysDen(3));
zeta                                    = shortPeriodSysDen(2)/(2*omegaN);

figure
poleZeroMap                             = pzplot(shortPeriodSys);
grid on
axis equal

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

%% Generate root locus study
figure
[r,k] = rlocus(-actuatorSys*shortPeriodSys);
rlocus(-actuatorSys*shortPeriodSys)
grid on
axis equal

hold on
scatter([real(p1Good),real(p2Good)],[imag(p1Good),imag(p2Good)],'filled','MarkerFaceAlpha',0.3,...
            'MarkerFaceColor',[0.5,0.5,0.5],'MarkerEdgeColor',[0,0,0],'SizeData',20,'Marker','hexagram') 
legend({'Systems root locus','Optimum poles based on fingerprint rule'})

%% Visualization options
COLOR       = [0,0,0];
LINEWIDTH   = 2;
FONTSIZE    = 15;

%% Figures
simOut          = sim('SortPeriodSimulation.slx');
figure
plot(simOut.W.Time,simOut.W.Data)

figure
plot(simOut.Q.Time,simOut.Q.Data)

%% Run open loop simulation
systemsInitialConditions    = [0,1];
simOutOpenLoop              = sim('SortPeriodSimulationOpenLoop.slx');
figure
plot(simOutOpenLoop.W.Time,simOutOpenLoop.W.Data)

figure
plot(simOutOpenLoop.Q.Time,simOutOpenLoop.Q.Data)