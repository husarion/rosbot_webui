#!/usr/bin/env python
# license removed for brevity
import rospy
import roslaunch
from std_msgs.msg import Bool

process = 0


def callback(message):
    global process
    if message.data:
        print "Received request for gmapping restart"
        process.stop()
        process.start()


def talker():
    global process
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.Subscriber("/reset_map", Bool, callback)

    rospy.init_node('gmap_suprvisor')
    rate = rospy.Rate(10)  # 1hz

    package = 'gmapping'
    executable = 'slam_gmapping'
    node = roslaunch.core.Node(package,
                               executable,
                               name='gmapping_node',
                               args='_base_frame:=base_link _odom_frame:=odom _delta:=0.05 _xmin:=-25 _ymin:=-25 _xmax:=25 _ymax:=25 _maxUrange:=5 _map_update_interval:=1 _linearUpdate:=0.2 _angularUpdate:=0.1',
                               output='screen')

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    print "Started supervisor"
    print process.is_alive()
    while not rospy.is_shutdown():
        rate.sleep()

    process.stop()
    print "Stopped"


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
