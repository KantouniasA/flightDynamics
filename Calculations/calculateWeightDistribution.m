%% Calculate weight distribution

weightTotal     = 2440; % [kg]

%% Weight fractions

% Weight distribution
fractionFuselage        = 0.09;
fractionWing            = 0.07;
fractionHTail           = 0.015;
fractionVTail           = 0.006;
fractionUndercarriage   = 0.05;

weightWing              = weightTotal*(fractionWing + fractionFuselage);	% Used in cm calc
weightHTail             = weightTotal*fractionHTail;                        % Used in cm calc
weightVTail             = weightTotal*fractionVTail;                        % Used in cm calc 
weightUndercarriage   	= weightTotal*fractionUndercarriage;                % Used in cm calc

% Motor weight https://en.wikipedia.org/wiki/Pratt_%26_Whitney_Canada_PT6
weightEngine            = 183;                                              % Used in cm calc
fractionEngine          = weightEngine/weightTotal;

% Usefull weight
weightUsefull           = 770;                                              % Used in cm calc
fractionUsefull         = weightUsefull/weightTotal;

% Cabin weight
weightCabin           	=   weightTotal...
                            - weightUsefull...
                            - weightEngine...
                            - weightWing...
                            - weightHTail...
                            - weightVTail...
                            - weightUndercarriage;

fractionCabin           =  weightCabin/weightTotal;

% CoM's location
load('zeroAxisCoM.mat')
load('cabinCoM.mat')
load('underCarriageCoM.mat')
load('engineCoM.mat')

cabinCoMFromWing            = cabinCoM - zeroAxisCoM;
underCarriageCoMFromWing    = underCarriageCoM - zeroAxisCoM;
engineCoMFromWing           = engineCoM - zeroAxisCoM;
usefullLoadCoMFromWing      = cabinCoMFromWing;    

% Calculate planes center of mass
load('neutralPoint.mat')
stabilityMargin = 0.15;
centerOfMass    = stabilityMargin*aerodynamicChord - neutralPoint;