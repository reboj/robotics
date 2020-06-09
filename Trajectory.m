function Trajectory()
%close all;
%clc;
set(0,'DefaultFigureWindowStyle','docked')
%% SAWYER INTIALIZATION AND PLOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
view([150 20])
sawyer = SAWYER;
a = arduino('COM5','Mega2560'); % setup the arduino mega
baseOrigin = [-1.875, -0.6, 1.06];
sawyer.model.base = transl(baseOrigin); 
hold on; 

f_hdmi1 = [-0.95, -0.4, 1.115]; 
f_h1E   = [-1, -0.4, 1.115];
f_hdmi2 = [-0.95, -0.5, 1.115];
f_h2E   = [-1, -0.5, 1.115];
m_hdmi  = [-1.875, 0.05, 1.16];
f_h3E   = [-1.05, -0.4, 1.115];

%% PLOT ENVIRONMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
loadObject('HDMI(female)(1).ply',f_hdmi1)
hold on;
loadObject('HDMI(female)(2).ply',f_hdmi2)
hold on;
%hdmi_m1 = Cable('HDMIMale(green).ply',[-2.037,0.5146,1.3460]);
hdmi = Cable('HDMIMale(green).ply',[-2.037,0.5146,1.3460]);

loadObject('tableandDesktop(1).ply',[0.5, -0.55, 0.66]) 
hold on;
loadObject('table(arm)(2).ply',[-1.875, -0.4, 0.355])
hold on;
loadObject('PCtower(2).ply',[-0.75, -0.5, 0.925])
hold on;

 loadObject('keyboard3.ply',[0, -0.35, 0.8])
 hold on;
 loadObject('EmergencyLight.ply',[-1.65, 0.3, 1.06])
 hold on;
 loadObject('emergencyStop.ply',[-2.1, 0.3, 1.1])
 hold on;
 loadObject('safetyFence1.ply',[0.5, -1.25, 0.35])
 hold on;
 loadObject('safetyFence3.ply',[0.5, 1.75, 0.35])
 hold on;
 loadObject('safetyFence90d1.ply',[1.5, -0.5, 0.35])
 hold on;
 loadObject('safetyFence90d2.ply',[1.5, 1, 0.35])
 hold on;
 loadObject('safetyFence90d3.ply',[-2.75, -0.5, 0.35])
 hold on;
 loadObject('safetyFence90d4.ply',[-2.75, 1, 0.35])
 hold on;
 loadObject('safetyFence2.ply',[-2, -1.25, 0.35])
 hold on;
 loadObject('safetyFence4.ply',[-2, 1.75, 0.35])
 hold on;
 loadObject('mouse(2).ply',[0.65, -0.25, 0.75])
 hold on;
 loadObject('coffeeMug1.ply',[0.75, -0.5, 0.8])
 hold on;
 view([150 20])
 input('prepare')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%% Robotic Movement using RMRC  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 5;                                                                      % Total time (s)
deltaT = 0.1;                                                               % Control frequency
steps = t/deltaT;                                                           % No. of steps for simulation

%Initialization of q joints. 
q_intial = zeros(1,7);
p_start  = sawyer.model.fkine(q_intial);
q_start  = sawyer.model.ikcon(p_start);
p_end    = transl(f_h3E)*troty(pi/2);
q_end    = sawyer.model.ikcon(p_end);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
for i = 1:steps
    ebutton = readVoltage(a,'A0'); % for each loop read the voltage of the potentiometer
    sensor = readDigitalPin(a,'D8'); % for each loop read the digital pin
    
    if(arduinocheck(ebutton,sensor)) %function to break loop
        stop_current_pose = sawyer.model.getpos();
        sawyer.model.animate(stop_current_pose);
        drawnow()
        input('continue?')
        continue
    end
    
    q_matrix(i,:) = (1-s(i))*q_start + s(i)*q_end;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
    objMove = sawyer.model.fkine(q_matrix(i,:));
    hdmi.move(objMove);
    
    end

%Move Sawyer with RMRC.
q_entry = q_matrix(end,:);
position = sawyer.model.fkine(q_entry);

 for i = 1:steps
     
    ebutton = readVoltage(a,'A0'); % for each loop read the voltage of the potentiometer
    sensor = readDigitalPin(a,'D8'); % for each loop read the digital pin
    
    if(arduinocheck(ebutton,sensor)) %function to break loop
        stop_current_pose = sawyer.model.getpos();
        sawyer.model.animate(stop_current_pose);
        drawnow()
        input('Stop Triggered, Continue?')
        continue
    end 
     
    q_current = sawyer.model.ikcon(position);
    J         = sawyer.model.jacob0(q_current);
    desired_V = [0.015 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
    q_dot     = pinv(J) * desired_V;
    q_k       = q_current + (q_dot*deltaT)';
    sawyer.model.animate(q_k);
    drawnow(); 
    position  = sawyer.model.fkine(q_k);
    hdmi.move(position);
    q_k_Matrix(i,:) = q_k;
    
 end

 %Move Sawyer with RMRC.

position = sawyer.model.fkine(q_k_Matrix(end,:));
 for i = 1:steps
     
    ebutton = readVoltage(a,'A0'); % for each loop read the voltage of the potentiometer
    sensor = readDigitalPin(a,'D8'); % for each loop read the digital pin
    
    if(arduinocheck(ebutton,sensor)) %function to break loop
        stop_current_pose = sawyer.model.getpos();
        sawyer.model.animate(stop_current_pose);
        drawnow()
        input('continue?')
        continue
    end 
     
    q_current = sawyer.model.ikcon(position);
    J         = sawyer.model.jacob0(q_current);
    desired_V = [-0.05 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
    q_dot     = pinv(J) * desired_V;
    q_k       = q_current + (q_dot*deltaT)';
    sawyer.model.animate(q_k);
    drawnow(); 
    position  = sawyer.model.fkine(q_k);
    q_k_Matrix2(i,:) = q_k;
    
 end
 
 
s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
q_end = q_k_Matrix2(end,:);
for i = 1:steps
    
    ebutton = readVoltage(a,'A0'); % for each loop read the voltage of the potentiometer
    sensor = readDigitalPin(a,'D8'); % for each loop read the digital pin
    
    if(arduinocheck(ebutton,sensor)) %function to break loop
        stop_current_pose = sawyer.model.getpos();
        sawyer.model.animate(stop_current_pose);
        drawnow()
        input('continue?')
        continue
    end
    
    q_matrix(i,:) = (1-s(i))*q_end + s(i)*q_start;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
    objMove = sawyer.model.fkine(q_matrix(i,:));
    
end






end

function [out]= arduinocheck(a,b)

if (a < 2.5 || b == 1) % arduino potentiometer voltage less than 2.5, b is sensor detects something in this range (lower 10cm, upper 100cm or something)
    out = true;
else
    out = false; 
end
end