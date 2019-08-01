%% 2D Planar 2-link Robot Revolute Joint Motion Animating Script
% Caleb Bisig 3/10/19
% Usage:
%       Before running this code, first run a script that generates a 2-column list
%       of theta 1 and theta 2 values called ThetaList. The list length
%       should be equal to the number of animation frames used below,
%       determined by a Timestep variable and an EndTime variable.
% Required inputs:
%       Timestep - integer
%       EndTime - integer
%       ThetaList - a 2x(EndTime/Timestep) array where col.1 is theta 1
%                                                      col.2 is theta 2


%% Part 0: Defining constants
clc
clear

length_1 = 2; %Link 1 length in units
length_2 = 2; %Link 1 length in units
EndTime = 50; % Seconds of runtime to goal position
%% Part 1: Generating waypoint angles from inverse kinematics

%--------------------------------------------------------------------------
% -> Brief explanation for IK portion:
% For a 2-link like this, there are 2 possible solutions in IK for any point inside the far boundary:
% Case 1: Theta 2 could be positive (i.e. the arm folds elbow downwards, v shape)
% Case 2: Theta 2 could be negative (i.e. the arm folds elbow upwards, ^ shape)

% Note that in problem 1, joint limits are 0<=theta<=Pi. Thus, only case 1
% is relevant. The equations for this are:

%Theta2=acos((x^2 + y^2 - (length_1)^2 - (length_2)^2) / (2*length_1*length_2)) 
%Theta1=atan(y/x)-atan((length_2*sin(Theta2)) / (length_1+length_2*cos(Theta2)))

%Theta 2 must be found first, as it is needed to solve for theta 1.

%(We could have done this by hand, but the formulas are readily available online). 
% For this assignment's IK equations above, I referenced the following video tutorial:
% https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

%Now, on to calculation!

%--------------------------------------------------------------------------

% Our waypoints in order are (4,0);(3,2);(2,3);(0,3);(-3,1);(-2,3);(-3,1)


%WaypntList = [4,0;3,2;2,3;0,3;-3,1;-2,3;-3,1]; % List of all required waypoints
WaypntList = [4,0;-3,1]; % List of just first and last required waypoints for testing purposes.

ThetaPointList = zeros(length(WaypntList),2); %Will Store Theta1 and Theta2 waypoint values

for i = 1:length(WaypntList)
    
    %Current waypoint for evaluation:
    x = WaypntList(i,1);
    y = WaypntList(i,2);
    
    %Generate its IK angles solution (via slide 13 of lecture 4):
    %NOTE! Lecture has atan2(1,2) written as atan2(2,1) for some reason!
    %I have corrected and reversed this here!
    
    D=(x^2 + y^2 - (length_1)^2 - (length_2)^2) / (2*length_1*length_2);
    Theta2 = atan2(sqrt(1-D^2),D);
	Theta1 = atan2(y,x)-atan2(length_2*sin(Theta2),length_1+length_2*cos(Theta2));

    
    %Store those angles
    ThetaPointList(i,:) = [Theta1, Theta2];
    
end

%% Generating Waypoint Control Equations via Matrix Solver

%we want to control 7 positions, 2 velocities, and 2 accelerations = 11 parameters.

 %{
    Control list:

        theta1(0)
        theta1(0)_dot
        theta1(0)_dotdot
        theta1(1)
        theta1(2)
        theta1(3)
        theta1(4)
        theta1(5)
        theta1(6)
        theta1(7)
        theta1(7)_dot 
        theta1(7)_dotdot
%}
% Therefore we need an 11x11 M_matrix of times and an 11x1 b_vector of a values.
%(See lecture 11, slides 14-15 for details.)

%We will assume that for the purposes of the assignment, waypoints are evenly timed.
WTI(2) = EndTime/(length(WaypntList)-1); %Waypoint Time Increment (in seconds)
%NOTE: WTI(1)=0 because time series must start at 0 for waypoint 1.

for i=1:(length(WaypntList)) %Incrementing timestep
    
    %Step through to next time increment, starting at zero
    WTI(i)=(i-1)*WTI(2); %Set current waypoint's time, starting at zero
    
    for j=1:(length(WaypntList))*3 %Create 3 rows at a time per timestep
        M_Matrix(i*3-2,j)= WTI(i)^(j-1); % row 1: pos.
        M_Matrix(i*3-1,j)= (j-1)*WTI(i)^(j-2); % row 2: vel.
        M_Matrix(i*3,j)  = (j-2)*(j-1)*WTI(i)^(j-3); % row 3: acc.
    end
        
end
M_Matrix(2,1)=0; %Fixing resulting NaN's that should be zeros
M_Matrix(3,1)=0;
M_Matrix(3,2)=0;

%Conditions control vector for Theta1 & Theta2:
for i=1:(length(WaypntList)) %Incrementing timestep

    %Step through to next time increment, starting at zero
    WTI(i)=(i-1)*WTI(2); %Set current waypoint's time, starting at zero

        b_vector1(i*3-2,1)= ThetaPointList(i,1); % row 1: pos.
        b_vector1(i*3-1,1)= .1; % row 2: vel.
        b_vector1(i*3  ,1)= 0; % row 3: acc.

        b_vector2(i*3-2,1)= ThetaPointList(i,2); % row 1: pos.
        b_vector2(i*3-1,1)= .1; % row 2: vel.
        b_vector2(i*3  ,1)= 0; % row 3: acc.
        
end

%Finding our solution coefficients:

a_1 = inv(M_Matrix)*b_vector1;
a_2 = inv(M_Matrix)*b_vector2;


%Calculating required theta values
Timestep = .1; %second

indexCounter=1; %hold our place for adding to next row in theta lists
for Time = 0:Timestep:EndTime

    q_1=0; %empty q_1 and q_2 angles to restart sum
    q_2=0;
    for k=1:length(a_1) %Implementing final trajectory equation by "a" vectors
        q_1=q_1 + a_1(k)*Time^(k-1);
        q_2=q_2 + a_2(k)*Time^(k-1);
    end
    

ThetaList(indexCounter,1)=q_1;
ThetaList(indexCounter,2)=q_2;

indexCounter=indexCounter+1;

end

%Final output ThetaList contains all joint 1 in row 1 and joint 2 in row 2.



