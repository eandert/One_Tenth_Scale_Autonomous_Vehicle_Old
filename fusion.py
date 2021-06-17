import sys
sys.path.insert(1, '../ATLAS/src')

import sensor_calculation
import sensor_fusion


class Fusion:
    def __init__(self):
        self.fusion_average = sensor_fusion.SensorFusion(sensor_fusion.method_averageOfAllDetections)
        self.fusion_kalman = sensor_fusion.SensorFusion(sensor_fusion.method_dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections)

# Build the object where we store all the accuracy stats
trackAccuracy = TrackAccuracyObject()
trackAccuracy.trackingId = vehId
trackAccuracy.horizontalCrossSection = horizontalCrossSection