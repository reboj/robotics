function  retreat()
%RETREAT Summary of this function goes here
%   Detailed explanation goes here
 
%TEST
% position =  transl([-1.4, -0.4, 1])*troty(pi/2); 

for i = 1:steps
    q_current = robot.ikcon(position);
    J = robot.jacob0(q_current);
    desired_V = [0 -0.5 0  0  0.5  0 ]';
    q_dot     = pinv(J) * desired_V;
    q_k   = q_current + (q_dot*deltaT)';
    robot.animate(q_k);
    drawnow(); 
    position = robot.fkine(q_k);
 end
end

