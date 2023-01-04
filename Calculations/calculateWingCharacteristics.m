%% Calculate wing characteristics Piper M500
% Calcualtes the basic wing geometrical characteristics of the craft

%% Load data from pictures

% Data from front view
load('Yw1.mat')
Yw1Y                    =   Yw1(:,1);
load('Zw1.mat')
Zw1Z                    =   Zw1(:,2);
load('wing2ElevatorZDist.mat')
wing2ElevatorZDistZ     =   wing2ElevatorZDist(:,2);

% Data from top view
load('wingElevatorChord.mat')
wingElevatorChordX   	=	wingElevatorChord(:,1);
load('wingElevatorSpan.mat')
wingElevatorSpanY     	=   wingElevatorSpan(:,2);

% Data from right view
load('finChord.mat')
finChordX          =	finChord(:,1);
load('finSpan.mat')
finSpanZ                =   finSpan(:,2);


%% Calculate dimensions of interest

% Wing chords
Cw1             =   wingElevatorChordX(6) - wingElevatorChordX(1);
Cw2             =   wingElevatorChordX(5) - wingElevatorChordX(2);
Cw3             =   wingElevatorChordX(4) - wingElevatorChordX(3);

% Elevator chords
Ce1             =   wingElevatorChordX(10) - wingElevatorChordX(7);
Ce2             =   wingElevatorChordX(9) - wingElevatorChordX(8);

% Fin chords
Cf1             =   finChordX(4) - finChordX(2);
Cf2             =   finChordX(5) - finChordX(3);

% Wing offsets
Ow12            =   wingElevatorChordX(2) - wingElevatorChordX(1);
Ow13            =   wingElevatorChordX(3) - wingElevatorChordX(1);

% Elevetor offsets
Oe12            =   wingElevatorChordX(8) - wingElevatorChordX(7);

% Fin offsets
Of12            =   finChordX(3) - finChordX(2);

% Wing span
Sw1             =   wingElevatorSpanY(1);
Sw2             =   wingElevatorSpanY(2);
Sw3             =   wingElevatorSpanY(3);

% Elevator span
Se1             =   wingElevatorSpanY(4);

% Fin span
Sf1              =   finSpanZ(2) - finSpanZ(1);

% Elevator z distance related to wing
DweZ             = wing2ElevatorZDistZ(2) - wing2ElevatorZDistZ(1);

% Elevator x distance related to wing 
DweX             = wingElevatorChordX(7) - wingElevatorChordX(1);

% Fin x distance related to wing 
DwfX             = finChordX(2) - finChordX(1);

% Calculate dihedral angle
dihedralAngle    = atan2(Zw1Z(2)-Zw1Z(1),Yw1Y(2)-Yw1Y(1));

