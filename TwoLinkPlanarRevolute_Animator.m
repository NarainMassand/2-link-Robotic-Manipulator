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
hold off
figure(2)
for MovieClock=1:length(0:Timestep:EndTime)
    
    %Clear figure plot for next frame of animation
    clf(figure(2))
    
    %Increment theta selection forward by 1 time step
    Theta1=ThetaList(MovieClock,1);
    Theta2=ThetaList(MovieClock,2);
    
    %-----------------Getting endpoints from Theta 1 and 2--------------------------
    %------0R1----------------------------------------------------------------------
    %Joint 1 Rotation Matrix on its own (0R1), which is a function of theta 1 only.
    
    %NOTE: column 4 is (x;y;0;1) where x, y are the link 1 endpoints  relative to
    %the origin.
    Joint1RotMatrix = [ cos(Theta1) -sin(Theta1) 0 length_1*cos(Theta1);...
        sin(Theta1)  cos(Theta1) 0 length_1*sin(Theta1);...
        0            0           1 0                  ;...
        0            0           0 1                  ];
    
    %------1R2----------------------------------------------------------------------
    %Joint 2 Rotation Matrix on its own (1R2), which is a function of theta 2 only.
    
    %NOTE: column 4 is (x;y;0;1) where x, y are the link 2 endpoints  relative to
    %the link 1 endpoint coordinate frame. THIS IS MEANINGLESS WITHOUT THE MATRIX ABOVE!
    Joint2RotMatrix = [ cos(Theta2) -sin(Theta2) 0 length_2*cos(Theta2);...
        sin(Theta2)  cos(Theta2) 0 length_2*sin(Theta2);...
        0            0           1 0                  ;...
        0            0           0 1                  ];
    
    %------0R2----------------------------------------------------------------------
    %The full FWD Kinematics is [Joint1]*[Joint2] or 0R1*1R2 = (0R2)
    % which is a function of theta 1 and theta 2.
    
    %NOTE: column 4 is (x;y;0;1) where x, y are the link 2 endpoints  relative to
    %the origin coordinate frame.
    Fwd_Kin_TranslationMatrix = Joint1RotMatrix*Joint2RotMatrix
    
    %-----------------Plotting links as point pairs---------------------------------
    %-------------------------------------------------------------------------------
    %Plot link 1 as the origin coordinates to the link 1 endpoint found through 0R1
    plot([0, Joint1RotMatrix(1,4)],[0, Joint1RotMatrix(2,4)]);
    hold on
    
    %Plot link 2 as the same 0R1 coordinates used above to the link 2 endpoint found through 0R2 (that is, through 0R1*1R2)
    plot([Joint1RotMatrix(1,4), Fwd_Kin_TranslationMatrix(1,4)],[Joint1RotMatrix(2,4), Fwd_Kin_TranslationMatrix(2,4)]);
    axis([-5 5 -5 5])
    
    %Store link points in list [x1,y1,x2,y2] (This doesn't get used anywhere, 
    % Just like to record in case I need to play back later or check out bugs)
    Fwd_Kin_positions(MovieClock,1)= Joint1RotMatrix(1,4);
    Fwd_Kin_positions(MovieClock,2)= Joint1RotMatrix(2,4);
    Fwd_Kin_positions(MovieClock,3)= Fwd_Kin_TranslationMatrix(1,4);
    Fwd_Kin_positions(MovieClock,4)= Fwd_Kin_TranslationMatrix(2,4);
end
