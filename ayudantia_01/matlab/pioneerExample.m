%% Guia 1 ELO325 2021
clear;clc;close all;
disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);


if(clientID > -1) 
    disp('Connected');
    
% Set Tags, se definen y habilitan los objetos
    [returnCode,left_Motor]   = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking );
    [returnCode,right_Motor]  = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking );
    [returnCode,front_sensor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_blocking );
    [returnCode,camera]       = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking );
    
% Inicializacion
    % Motor: actuar sobre la velocidad del motor
    [returnCode] = sim.simxSetJointTargetVelocity(clientID, left_Motor , 0.2 , sim.simx_opmode_blocking);
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_Motor , 0.1 , sim.simx_opmode_blocking);
    % Proximity Sensor:
    [returnCode,detectionState,detectedPoint,~,~] = sim.simxReadProximitySensor(clientID,front_sensor,sim.simx_opmode_streaming);
    % Vision Sensor:
    [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_streaming);
    
% Buffer, lectura de datos
    for i=1:50
        % Proximity Sensor
        [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,front_sensor,sim.simx_opmode_buffer);
        disp(norm(detectedPoint));
        
        % Vision Sensor
        [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_buffer);
        imshow(image)
        
        pause(0.1)
    end
 
% Finalizacion
    [returnCode] = sim.simxSetJointTargetVelocity(clientID, left_Motor , 0 , sim.simx_opmode_blocking);
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_Motor , 0 , sim.simx_opmode_blocking); 
    sim.simxFinish(-1);
end
    
sim.delete();