%% Simulations in order to obtain stable aircraft with correct cl
% Calculate cl needed
areaRef         = 19.689; % [m^2]
g               = 9.81;
airDensity      = 1.225;  % [kg/m^3]
aircraftMass    = 2439;   % [kg]
aircraftSpeed   = 133;    % [m/s]

liftNeeded      = aircraftMass*g;                   % [N]
dynamicPressure = 1/2*airDensity*aircraftSpeed^2;   % [Pa]
clNeeded        = liftNeeded/dynamicPressure/areaRef;

% Initialize simulation structure
Simulations     = struct;

%% Read graph results for CoM related simulation

% 1st simulation
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T1-133_0 m_s-VLM1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.CoM.Simulation2.elevatorTilt        = 0;
Simulations.CoM.Simulation2.wingTilt            = 0;
Simulations.CoM.Simulation2.cabinCenterOfMass   = 2.2;
Simulations.CoM.Simulation2.resultsTable        = resultsTable;

% 2nd simulation
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T2-133_0 m_s-VLM1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.CoM.Simulation1.elevatorTilt        = 0;
Simulations.CoM.Simulation1.wingTilt            = 0;
Simulations.CoM.Simulation1.cabinCenterOfMass   = 2.7;
Simulations.CoM.Simulation1.resultsTable        = resultsTable;

% 3nd simulation
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T3-133_0 m_s-VLM1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.CoM.Simulation3.elevatorTilt        = 0;
Simulations.CoM.Simulation3.wingTilt            = 0;
Simulations.CoM.Simulation3.cabinCenterOfMass   = 1.7;
Simulations.CoM.Simulation3.resultsTable        = resultsTable;

%% Read graph results for elevator and wing angle related simulation
% Copy simulation 3 to elevator angle simulation 1 data
Simulations.EleAng.Simulation2                  = Simulations.CoM.Simulation3;

% Simulation 3 with wing tilt = -1
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T3 Wing angle minus 1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.EleAngWT.Simulation2.elevatorTilt        = 0;
Simulations.EleAngWT.Simulation2.wingTilt            = -1;
Simulations.EleAngWT.Simulation2.cabinCenterOfMass   = 1.7;
Simulations.EleAngWT.Simulation2.resultsTable        = resultsTable;

% 4th simulation
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T4-133_0 m_s-VLM1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.EleAng.Simulation1.elevatorTilt        = -1;
Simulations.EleAng.Simulation1.wingTilt            = 0;
Simulations.EleAng.Simulation1.cabinCenterOfMass   = 1.7;
Simulations.EleAng.Simulation1.resultsTable        = resultsTable;

% Simulation 4 with wing tilt = -1
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T4 Wing angle minus 1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.EleAngWT.Simulation1.elevatorTilt        = -1;
Simulations.EleAngWT.Simulation1.wingTilt            = -1;
Simulations.EleAngWT.Simulation1.cabinCenterOfMass   = 1.7;
Simulations.EleAngWT.Simulation1.resultsTable        = resultsTable;

% 5th simulation
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T5-133_0 m_s-VLM1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.EleAng.Simulation3.elevatorTilt        = 1;
Simulations.EleAng.Simulation3.wingTilt            = 0;
Simulations.EleAng.Simulation3.cabinCenterOfMass   = 1.7;
Simulations.EleAng.Simulation3.resultsTable        = resultsTable;

% Simulation 5 with wing tilt = -1
dataPath = "C:\Users\Antonis Kantounias\Documents\ergasies\flightDynamics\Calculations\Extracted data\T5 Wing angle minus 1.csv";
resultsTable   = readtable(dataPath, 'VariableNamingRule', 'preserve');
Simulations.EleAngWT.Simulation3.elevatorTilt        = 1;
Simulations.EleAngWT.Simulation3.wingTilt            = -1;
Simulations.EleAngWT.Simulation3.cabinCenterOfMass   = 1.7;
Simulations.EleAngWT.Simulation3.resultsTable        = resultsTable;


%% Plot options
colorPalet1         = 	[64,224,208]/255;
colorPalet2         = 	[75,0,130]/255;
FONTSIZE            =   15;
FIG_POSITION        =   [50,50,1100,500];
FIG_POSITION2       =   [50,50,1500,700];

%% Generate angle - static stability plots
simulationNames     =   fieldnames(Simulations.CoM);
numOfSimulaitions   =   length(simulationNames);
figCoM = figure;
figCoM.Position = FIG_POSITION;
grid on;
grid minor;
titleMsg    = sprintf('CM vs Plane angle of attack\nCabin CoM study');
title(titleMsg,'fontsize',FONTSIZE);
xlabel('a [deg]','fontsize',FONTSIZE);
ylabel('Cm','fontsize',FONTSIZE);
legendMsg = cell(1,numOfSimulaitions);

for iSimulation = 1:numOfSimulaitions
    % Load plot data
    colorWeight             = (iSimulation-1)/(numOfSimulaitions-1);
    Simulation              = Simulations.CoM.(['Simulation',num2str(iSimulation)]);
    resultsTable            = Simulation.resultsTable;
    cabinCenterOfMass       = Simulation.cabinCenterOfMass;
  	elevatorTilt            = Simulation.elevatorTilt;
    
    % Generate plot
    colorPalet              = (1-colorWeight)*colorPalet1 + colorWeight*colorPalet2;
    legendMsg{iSimulation} 	= sprintf([ 'Cabin CoM x position = ', num2str(cabinCenterOfMass), ' [m]', '\n'...
                                        'Elevator tilt angle = ', num2str(elevatorTilt), ' [deg]', '\n'...
                                        'Wing tilt angle = ',num2str(wingTilt), ' [deg]','\n']);
    hold on
    plot(resultsTable.alpha,resultsTable.Cm,'linewidth',2,'color',colorPalet)
    hold off
end

legend(legendMsg,'fontsize',FONTSIZE,'location','northeastoutside');

%% Generate elevator and wing tilt angle static stability plots
figEleAng = figure;
figEleAng.Position = FIG_POSITION2;
tiledlayout(1,2)
% ------------------------------------------------%

tCM = nexttile;
grid on;
grid minor;

simulationNames     =   fieldnames(Simulations.EleAng);
numOfSimulaitions   =   length(simulationNames);

titleMsg    = sprintf('CM vs Plane angle of attack\nElevator and wing tilt angle study');
title(tCM,titleMsg,'fontsize',FONTSIZE);
xlabel(tCM,'a [deg]','fontsize',FONTSIZE);
ylabel(tCM,'Cm','fontsize',FONTSIZE);
legendMsg1 = cell(1,numOfSimulaitions);

for iSimulation = 1:numOfSimulaitions
    % Load plot data
    colorWeight             = (iSimulation-1)/(numOfSimulaitions-1);
    Simulation              = Simulations.EleAng.(['Simulation',num2str(iSimulation)]);
    resultsTable            = Simulation.resultsTable;
    cabinCenterOfMass       = Simulation.cabinCenterOfMass;
  	elevatorTilt            = Simulation.elevatorTilt;
    wingTilt                = Simulation.wingTilt;
    
    % Generate plot
    colorPalet              = (1-colorWeight)*colorPalet1 + colorWeight*colorPalet2;
    legendMsg1{iSimulation} 	= sprintf([ 'Cabin CoM x position = ', num2str(cabinCenterOfMass), ' [m]', '\n'...
                                        'Elevator tilt angle = ', num2str(elevatorTilt), ' [deg]', '\n'...
                                        'Wing tilt angle = ',num2str(wingTilt), ' [deg]','\n']);
    hold on
    plot(tCM,resultsTable.alpha,resultsTable.Cm,'linewidth',2,'color',colorPalet)
    hold off
end

% ------------------------------------------------%
simulationNames     =   fieldnames(Simulations.EleAngWT);
numOfSimulaitions   =   length(simulationNames);
legendMsg2 = cell(1,numOfSimulaitions);

for iSimulation = 1:numOfSimulaitions
    % Load plot data
    colorWeight             = (iSimulation-1)/(numOfSimulaitions-1);
    Simulation              = Simulations.EleAngWT.(['Simulation',num2str(iSimulation)]);
    resultsTable            = Simulation.resultsTable;
    cabinCenterOfMass       = Simulation.cabinCenterOfMass;
  	elevatorTilt            = Simulation.elevatorTilt;
    wingTilt                = Simulation.wingTilt;
    
    % Generate plot
    colorPalet              = (1-colorWeight)*colorPalet1 + colorWeight*colorPalet2;
    legendMsg2{iSimulation} 	= sprintf([ 'Cabin CoM x position = ', num2str(cabinCenterOfMass), ' [m]', '\n'...
                                        'Elevator tilt angle = ', num2str(elevatorTilt), ' [deg]', '\n'...
                                        'Wing tilt angle = ',num2str(wingTilt), '[deg]', '\n']);
    hold on
    plot(tCM,resultsTable.alpha,resultsTable.Cm,'linewidth',2,'color',colorPalet,'linestyle','--')
    hold off
end

%%------------------------------------------------%%

tCL = nexttile;
simulationNames     =   fieldnames(Simulations.EleAng);
numOfSimulaitions   =   length(simulationNames);

grid on;
grid minor;
titleMsg    = sprintf('CL vs Plane angle of attack\nElevator and wing tilt angle study');
title(tCL,titleMsg,'fontsize',FONTSIZE);
xlabel(tCL,'a [deg]','fontsize',FONTSIZE);
ylabel(tCL,'CL','fontsize',FONTSIZE);
legendMsg1 = cell(1,numOfSimulaitions);

for iSimulation = 1:numOfSimulaitions
    % Load plot data
    colorWeight             = (iSimulation-1)/(numOfSimulaitions-1);
    Simulation              = Simulations.EleAng.(['Simulation',num2str(iSimulation)]);
    resultsTable            = Simulation.resultsTable;
    cabinCenterOfMass       = Simulation.cabinCenterOfMass;
  	elevatorTilt            = Simulation.elevatorTilt;
    wingTilt                = Simulation.wingTilt;
    
    % Generate plot
    colorPalet              = (1-colorWeight)*colorPalet1 + colorWeight*colorPalet2;
    legendMsg1{iSimulation} 	= sprintf([ 'Cabin CoM x position = ', num2str(cabinCenterOfMass), ' [m]', '\n'...
                                        'Elevator tilt angle = ', num2str(elevatorTilt), ' [deg]', '\n'...
                                        'Wing tilt angle = ',num2str(wingTilt), ' [deg]','\n']);
    hold on
    plot(tCL,resultsTable.alpha,resultsTable.CL,'linewidth',2,'color',colorPalet)
    hold off
end

% ------------------------------------------------%
simulationNames     =   fieldnames(Simulations.EleAngWT);
numOfSimulaitions   =   length(simulationNames);
legendMsg2 = cell(1,numOfSimulaitions);

for iSimulation = 1:numOfSimulaitions
    % Load plot data
    colorWeight             = (iSimulation-1)/(numOfSimulaitions-1);
    Simulation              = Simulations.EleAngWT.(['Simulation',num2str(iSimulation)]);
    resultsTable            = Simulation.resultsTable;
    cabinCenterOfMass       = Simulation.cabinCenterOfMass;
  	elevatorTilt            = Simulation.elevatorTilt;
    wingTilt                = Simulation.wingTilt;
    
    % Generate plot
    colorPalet              = (1-colorWeight)*colorPalet1 + colorWeight*colorPalet2;
    legendMsg2{iSimulation} 	= sprintf([ 'Cabin CoM x position = ', num2str(cabinCenterOfMass), ' [m]', '\n'...
                                        'Elevator tilt angle = ', num2str(elevatorTilt), ' [deg]', '\n'...
                                        'Wing tilt angle = ',num2str(wingTilt), '[deg]', '\n']);
    hold on
    plot(tCL,resultsTable.alpha,resultsTable.CL,'linewidth',2,'color',colorPalet,'linestyle','--')
    hold off
end

hold on
legendMsg2{iSimulation+1}   = sprintf(['At a = 0 [deg],',' Cl needed = ',num2str(clNeeded)]);
plot(tCL,resultsTable.alpha([1,end]),[clNeeded,clNeeded],'linewidth',1,'color',[1,0,0],'linestyle','-.')
hold off

% ------------------------------------------------%

legend(tCL,[legendMsg1,legendMsg2],'fontsize',FONTSIZE,'location','northeastoutside');
