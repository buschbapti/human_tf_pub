#!/usr/bin/env python
import rospy
import rospkg
import tf
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,Pose
from multiprocessing import Lock
from os.path import join
from os.path import exists
from os import makedirs
import csv


class Recorder(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.data_dir = join(rospack.get_path("human_tf_pub"), "data")
        if not exists(self.data_dir):
            makedirs(self.data_dir)

        self.killed = False
        self.recording = False
        self.got_header = False
        self.header = []
        self.recorded_data = []
        self.lock = Lock()
        self.pose_subscriber = rospy.Subscriber("/box/pose", PoseStamped, self.process_box_pose)
        self.last_box_pose = Pose()
        self.joint_subscriber = rospy.Subscriber("/human/set_joint_values", JointState, self.process_human_joints)
        self.last_human_joint_state = JointState()

    def init_header(self,names):
        self.header = names
        self.header.append('box_x')
        self.header.append('box_y')
        self.header.append('box_z')

    def process_box_pose(self,msg):
        with self.lock:
            self.last_box_pose = msg.pose

    def process_human_joints(self,msg):
        with self.lock:
            # Initialize the names of the joints for saving to file
            if not self.got_header:
                self.init_header(msg.name)
                self.got_header= True
            self.last_human_joint_state = msg
    

    def start_recording(self):
        with self.lock:
            self.recording = True
            self.recorded_data = []

    def stop_recording(self, savefile):
        savefile = join(self.data_dir, savefile + ".csv")
        with self.lock:
            self.recording = False
            with open(savefile, 'w') as f:
                writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(self.header)
                for row in self.recorded_data:
                    writer.writerow(row)
        return savefile

    def record_transforms(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and not self.killed:
            with self.lock:
                if self.recording:
                    data = list(self.last_human_joint_state.position)
                    data.append(self.last_box_pose.position.x)
                    data.append(self.last_box_pose.position.y)
                    data.append(self.last_box_pose.position.z)
                    #print(data)
                    if len(data) > 3:
                        self.recorded_data.append(data)
                     #   print(self.recorded_data)
            rate.sleep()

    def run(self):
        thread = threading.Thread(target=self.record_transforms)
        thread.start()

    def stop(self):
        self.killed = True

