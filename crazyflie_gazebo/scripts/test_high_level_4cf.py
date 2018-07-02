#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('test_high_level_4cf')

    cf1 = crazyflie.Crazyflie("cf1", "/cf1")
    cf2 = crazyflie.Crazyflie("cf2", "/cf2")
    cf3 = crazyflie.Crazyflie("cf3", "/cf3")
    cf4 = crazyflie.Crazyflie("cf4", "/cf4")

    cf1.setParam("commander/enHighLevel", 1)
    cf2.setParam("commander/enHighLevel", 1)
    cf3.setParam("commander/enHighLevel", 1)
    cf4.setParam("commander/enHighLevel", 1)

    cf1.takeoff(targetHeight = 0.5, duration = 10.0)
    cf2.takeoff(targetHeight = 0.5, duration = 10.0)
    cf3.takeoff(targetHeight = 0.5, duration = 10.0)
    cf4.takeoff(targetHeight = 0.5, duration = 10.0)
    time.sleep(10.0)

    # cf.land(targetHeight = 0.0 , duration = 4.0)
    # time.sleep(20.0)
    # rospy.spin()
    cf1.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    cf2.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    cf3.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    cf4.goTo(goal = [0.5, 0.0, 0.0], yaw=0.2, duration = 2.0, relative = True)
    time.sleep(10.0)

    cf1.land(targetHeight = 0.0, duration = 2.0)
    cf2.land(targetHeight = 0.0, duration = 2.0)
    cf3.land(targetHeight = 0.0, duration = 2.0)
    cf4.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(15.0)
    
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("takeoff.csv")

    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("figure8.csv")

    print(traj1.duration)

    cf1.uploadTrajectory(0, 0, traj1)
    cf1.uploadTrajectory(1, len(traj1.polynomials), traj2)
    cf2.uploadTrajectory(0, 0, traj1)
    cf2.uploadTrajectory(1, len(traj1.polynomials), traj2)
    cf3.uploadTrajectory(0, 0, traj1)
    cf3.uploadTrajectory(1, len(traj1.polynomials), traj2)
    cf4.uploadTrajectory(0, 0, traj1)
    cf4.uploadTrajectory(1, len(traj1.polynomials), traj2)

    cf1.startTrajectory(0, timescale=1.0)
    cf2.startTrajectory(0, timescale=1.0)
    cf3.startTrajectory(0, timescale=1.0)
    cf4.startTrajectory(0, timescale=1.0)
    time.sleep(traj1.duration * 2.0)

    cf1.startTrajectory(1, timescale=2.0)
    cf2.startTrajectory(1, timescale=2.0)
    cf3.startTrajectory(1, timescale=2.0)
    cf4.startTrajectory(1, timescale=2.0)
    time.sleep(traj2.duration * 2.0)

    cf1.startTrajectory(0, timescale=1.0, reverse=True)
    cf2.startTrajectory(0, timescale=1.0, reverse=True)
    cf3.startTrajectory(0, timescale=1.0, reverse=True)
    cf4.startTrajectory(0, timescale=1.0, reverse=True)
    time.sleep(traj1.duration * 1.0)

    cf1.stop()
    cf2.stop()
    cf3.stop()
    cf4.stop()
