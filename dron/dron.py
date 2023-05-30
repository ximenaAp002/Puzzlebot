#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform, Pose
import math as m

odometry = Pose()
xrd=[]
yrd=[]
zrd=[]

def points_generator():
    for i in range(100):
        xrd.append(2.5*m.cos(i/10))
        yrd.append(2.5*m.sin(i/10))
        zrd.append(i/10)
    

def odometry_callback(data):
    global odometry
    odometry = data.position

def main():
    rospy.init_node('node_example')
    nodeRate = 100
    rate = rospy.Rate(nodeRate)
    waypoint_publisher = rospy.Publisher('/hummingbird/command/trajectory',  MultiDOFJointTrajectory, queue_size= 10)
    odometry = rospy.wait_for_message('/hummingbird/odometry_sensor1/pose', Pose, timeout=5)
    #sub_odometry = rospy.Subscriber('/hummingbird/odometry_sensor1/pose', Pose , odometry_callback)
    #generar sussribver que escuche odometry
    #encuentre la posiscion actual y ese sea el punto inicial de la trayectoria
    #odometru sensor
    #odometry ground truth 


    drone_msg = MultiDOFJointTrajectory()
    drone_msg.header.stamp = rospy.Time.now()
    drone_msg.header.frame_id = 'base link'
    drone_msg.joint_names.append('base link')

    #creat starting point
    transforms =Transform()
    velocities = Twist()
    acceleration = Twist()


    transforms.translation.x = 0.0
    print(odometry.position.x)  
    transforms.translation.y = 0.0
    print(odometry.position.y)
    transforms.translation.z = 1.0  
    print(odometry.position.z)

    point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [acceleration], rospy.Time(1))
    drone_msg.points.append(point)

    points_generator()
    #create end point
   
    
    while not rospy.is_shutdown():
        transforms = Transform()
        for i in range(len(xrd)):
            print(i)
            transforms.translation.x = xrd[i]
            transforms.translation.y = yrd[i]
            transforms.translation.z = zrd[i]

            velocities = Twist()
            acceleration = Twist()
            point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [acceleration], rospy.Time(2))
            drone_msg.points.append(point)
            waypoint_publisher.publish(drone_msg)

        transforms.rotation.w = 1.0
        transforms.rotation.x = 0.0
        transforms.rotation.y = 0.0
        transforms.rotation.z = 0.0

        odometry = rospy.wait_for_message('/hummingbird/odometry_sensor1/pose', Pose, timeout=5)
        
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
