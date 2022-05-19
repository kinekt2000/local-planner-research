#!/usr/bin/env python

from time import sleep
import rospy
import rosgraph_msgs.msg
import subprocess
import os
from pprint import pprint


class RosbagRecord:
    def __init__(self, record_script, record_folder):

        if record_script and record_folder:
            self.started = False
            rospy.Subscriber("/rosout", rosgraph_msgs.msg.Log, lambda data: self.check_status(data))

            rospy.loginfo(rospy.get_name() + ' starts rosbag')
            self.record_script = record_script
            self.record_folder = record_folder
            rospy.on_shutdown(self.stop_recording_handler)

            # Start recording.
            command = self.record_script
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                                      executable='/bin/bash')

            while not self.started:
                rospy.loginfo(rospy.get_name() + " is waiting for recording start")
                sleep(0.5)
            rospy.loginfo(rospy.get_name() + " started recording")

        else:
            rospy.signal_shutdown(rospy.get_name() + ' no record script or folder specified.')

    def check_status(self, data):
        if not self.started:
            self.started = "Recording to" in data.msg
        return self.started

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split(b"\n"):
            if (str.startswith(s)):
                os.system(b"rosnode kill " + str)

    def stop_recording_handler(self):
        rospy.loginfo(rospy.get_name() + ' stop recording.')
        self.terminate_ros_node(b"/record")

if __name__ == '__main__':
    rospy.init_node('rosbag_record')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rosbag_record = RosbagRecord(
            rospy.get_param('~record_script'),
            rospy.get_param('~record_folder')
        )
    except rospy.ROSInterruptException:
        pass