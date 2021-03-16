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
response = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# Now that we have chatted with the RSU server, we should know where we are going
planner = planning_control.Planner()
planner.initialVehicleAtPosition(response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"], response["route_TFL"], vehicle_id, False)

# Fails keeps track of how many tries to connect with 
fails = 0

while True:
    # This will block until our LIDAR data is available on the pipes
    try:
         lidarDevice.checkFromC()
    except Exception as e:
        print ( " Lidar timed out, ", str(e) )
        # If we timed out we need to reconect to the LIDAR and try everything again
        # Make sure we run an emergency stop to make sure we dont keep driving blind
        egoVehicle.emergencyStop()
        lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)
        continue

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
            #vehicleObservations = fusion.KalmanFilter(lidarcoordinates, lidartimestamp, camcoordinates, camtimestamp)

            # Message the RSU, for now we must do this before our control loop
            # as the RSU has the traffic light state information
            objectPackage = {
                "lidar_t":lidartimestamp,
                "lidar_obj":lidarcoordinates,
                "cam_t":camtimestamp,
                "cam_obj":camcoordinates,
                "fused_t":"null",
                "fused_obj":"null",
            }
            response_checkin = rsu.checkin(vehicle_id, lidarDevice.localizationX, lidarDevice.localizationY, 0.0, 0.0, 0.0, lidarDevice.localizationYaw, objectPackage)
            #tflState = rsu.messageLocation(vehicle_id, lidartimestamp, lidarDevice.localization, vehicleObservations)

            # Check if our result is valid
            if response_checkin == None:
                # Cut the engine to make sure that we don't hit anything since the central controller is down
                print ( "Error: RSU response not recieved in time, stopping" )
                egoVehicle.emergencyStop()
                if fails < 20:
                    fails += 1
                else:
                    print ( "Attempting to re-register with RSU" )
                    # We have failed a lot lets try to re-register but use our known location
                    response = rsu.register(vehicle_id, lidarDevice.localizationX, lidarDevice.localizationY, 0.0, 0.0, 0.0, lidarDevice.localizationYaw)
            else:
                # Update our various pieces
                print ( "v_t ", float(response_checkin["v_t"]) )
                planner.targetVelocityGeneral = float(response_checkin["v_t"])
                planner.update_localization([lidarDevice.localizationX, lidarDevice.localizationY, lidarDevice.localizationYaw])
                print ( lidarDevice.localizationX, lidarDevice.localizationY, lidarDevice.localizationYaw )
                planner.recieve_coordinate_group_commands(response_checkin["tfl_state"])
                planner.pure_pursuit_control()

                # Now update our current PID with respect to other vehicles
                planner.check_positions_of_other_vehicles_adjust_velocity(response_checkin["veh_locations"], vehicle_id)
                # We can't update the PID controls until after all positions are known
                planner.update_pid()
                # Finally, issue the commands to the motors
                steering_ppm, motor_pid = planner.return_command_package()
                egoVehicle.setControlMotors(steering_ppm, motor_pid)

                fails = 0

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

egoVehicle.emergencyStop()

client.close()
server.close()

plt.show()
