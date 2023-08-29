import sim              # Python wrapper for CoppeliaSim
import sys              # System calls library
import numpy as np      # Library to work with matrices (like Matlab)
import cv2              # Computer Vision Library

sim.simxFinish(-1)

clientID = sim.simxStart(connectionAddress='127.0.0.1',
                         connectionPort=19990,
                         waitUntilConnected=True,
                         doNotReconnectOnceDisconnected=True,
                         timeOutInMs=5000,
                         commThreadCycleInMs=5)

if clientID!= -1:
    print("Connected to Remote API Server")
else:
    print("Connection failed")
    sys.exit('Could not reconnect')

errorcode,left_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_oneshot_wait)
errorcode,right_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)
print("Given Target Velocities to the Joints")
errorcode,cam_handle = sim.simxGetObjectHandle(clientID,'Cam',sim.simx_opmode_oneshot_wait)
try:
    while True:

        sim.simxSetJointTargetVelocity(clientID,left_motor_handle,1,sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID,right_motor_handle,1,sim.simx_opmode_streaming)

        errorCode,resolution,image=sim.simxGetVisionSensorImage(clientID,cam_handle,0,sim.simx_opmode_streaming)
        if len(image)>0:
            image = np.array(image,dtype=np.dtype('uint8'))
            image = np.reshape(image,(resolution[1],resolution[0],3))
            cv2.imshow("Image", image)
            cv2.waitKey(10)
except KeyboardInterrupt:   #Checks if ctrl+c is pressed 
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print("Stopping Simulation")
    pass
    
sim.simxFinish(clientID)