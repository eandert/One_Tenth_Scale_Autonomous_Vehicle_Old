import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue

import camera_recognition
import lidar_recognition
import planning_control
import communication
import motors

pipeFromC= "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"
pipeToC= "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc"

vehicle_id = 0

def processImagesThread(q, oq, settings, camSpecs):
    # Init the camera class
    cameraRecognition = camera_recognition.Camera(settings, camSpecs)
    queueId = 0

    # Now wait for input
    while 1:
        if not q.empty():
            goSign = q.get()[0]
            if goSign != (queueId + 1):
                print ( " Error frame mistmatch in camera queue. " , goSign, " != ", queueId + 1 )
            # Process the camera frame
            camcoordinates, camtimestamp = cameraRecognition.takeCameraFrame(settings,camSpecs)
            # Put the results back in the queue for the main thread
            oq.put([camcoordinates, camtimestamp, goSign])

debug = False

# The first thing we should always do is initialize the control module
# This is important to make sure a rogue signal doesn't drive us away
egoVehicle = motors.Motors()

# Start the connection with the LIDAR through pipes
lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)

# Init the camera
# Setup our various settings
settings = camera_recognition.Settings()
settings.darknetPath = '../darknet/'
camSpecs = camera_recognition.CameraSpecifications()
camSpecs.cameraHeight = .2
camSpecs.cameraAdjustmentAngle = 0.0
frameID = 1

# Spawn the camera processing thread
q = Queue()
oq = Queue()
cameraThread = Process(target=processImagesThread, args=(q, oq, settings, camSpecs))
cameraThread.start()

# Wait to make sure that we have started YOLO
time.sleep(10)

# Init the LIDAR processing class
lidarRecognition = lidar_recognition.LIDAR(time.time())

# Start the connection with the RSU (Road Side Unit) server through sockets
rsu = communication.connectServer()
rsu.register(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
#x_offset, y_offset, theta_offset, xCoordinates, yCoordinates, vCoordinates, id_in, simVehicle = rsu.requestRoute()

# Now that we have chatted with the RSU server, we should know where we are going
planner = planning_control.Planner()
#planner.initialVehicleAtPosition(x_offset, y_offset, theta_offset, xCoordinates, yCoordinates, vCoordinates, id_in, simVehicle)

while True:
    # This will block until our LIDAR data is available on the pipes
    lidarDevice.checkFromC()
    # Now that we have LDIAR data, signal the process to start the camera processing
    q.put([frameID])
    
    # Process the LIDAR while we are processign the camera in the background
    lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(), lidarDevice.time)
    
    # Now get the result from the other thread and check it is valid
    returned = oq.get()
    if len(returned) == 3:
        camcoordinates = returned[0]
        camtimestamp = returned[1]
        frameIDSent = returned[2]
        # Check that the order has been kept and that we processed the correct frame
        if frameID != frameIDSent:
            print ( " Error, mismatched camera frame returned ", frameID, " != ", frameIDSent )
            # Cut the engine to make sure that we don't hit anything since we are blind
            egoVehicle.emergencyStop()
        else:
            vehicleObservations = fusion.KalmanFilter(lidarcoordinates, lidartimestamp, camcoordinates, camtimestamp)

            # Message the RSU, for now we must do this before our control loop
            # as the RSU has the traffic light state information
            tflState = rsu.messageLocation(vehicle_id, lidartimestamp, lidarDevice.localization, vehicleObservations)

            # Update our various pieces
            planner.update_localization(lidarDevice.localization)
            planner.recieve_coordinate_group_commands(tflState)
            planner.pure_pursuit_control()

            # Now update our current PID with respect to other vehicles
            planner.check_positions_of_other_vehicles_adjust_velocity(vehicleObservations, vehicle_id)
            # We can't update the PID controls until after all positions are known
            planner.update_pid()
            # Finally, issue the commands to the motors
            egoVehicle.setControlMotors(planner.return_command_package)

            if debug:
                plt.cla()
                # Create plot for lidar and camera points
                for i, data in enumerate(lidarcoordinates[:]):
                    plt.plot(data[1], data[2], 'go')
                    plt.annotate(data[0], (data[1], data[2]))
                for i, data in enumerate(camcoordinates):
                    plt.plot(data[1], data[2], 'ro')
                    plt.annotate(data[0], (data[1], data[2]))
                plt.title("Sensors")
                plt.pause(0.001)

            print ( " Time taken: " , time.time() - lidartimestamp, time.time() )
    else:
        print ( " Error, no camera frame returned " )
        # Cut the engine to make sure that we don't hit anything since we are blind
        egoVehicle.emergencyStop()

plt.show()

egoVehicle.emergencyStop()

client.close()
server.close()
