function MoveSawyer(self,robot)
%% integrate the ply files here + functions to move the sawyer. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Moving Sawyer from A to B. After Altering qlim of Sawyer in DH.  
% trajSteps = 50;
% q = zeros(1,7);                                                            % Starting q config.
% transEnd = transl(0.5,-0.54,-0.203);                                       % Desired position.
% qEnd = robot.ikcon(transEnd)
% qMatrix = jtraj(q,qEnd,trajSteps)
% for i = 1:trajSteps
%     robot.animate(qMatrix(i,:));                                           % "Grab" an object in this for loop through
%     drawnow();
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%"PUSH/PULL MOVEMENT" once robot end-effector is in line with female port
%%% RMRC could also be a solution for this. 
% for y = -0.5:0.05:0.5
%     newQ = robot.ikine(transl(-0.75,y,0),newQ,[1,1,0,0,0,0]);%,'alpha',0.01);
%     robot.animate(newQ);
%     drawnow();
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Refer to Quiz4Solu1 for this ^^. Basically more iteration, the straighter the movement.
end

