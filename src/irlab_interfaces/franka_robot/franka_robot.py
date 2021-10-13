#! /usr/bin/env python
from __future__ import absolute_import
from panda_robot import PandaArm
import argparse
import copy
import rospy
import numpy as np
import quaternion
from geometry_msgs.msg import WrenchStamped

log_once = True


class FrankaRobot(PandaArm, RobotInterface):
    """
        ROS Python Interface class for Franka Emika Panda robot relying on panda_robot.PandaArm
        (https://www.github.com/justagist/panda_robot)
    """

    def __init__(self, limb = None, on_state_callback = None, reset_frames = True):

        RobotInterface.__init__(self)
        self._log_count = 0
        PandaArm.__init__(self, on_state_callback=on_state_callback, reset_frames=reset_frames)

    def _configure(self, on_state_callback):

        self._transform_ft_vals = False # transform forces from FT sensor frame to base frame (no need; already in base frame)
        self._smooth_ft_vals = True

        # self._ft = ft_sensor.FTSensor(ip=get_ip(),imu="use_mock") # Don't use it directly here; causes issues when multiple instance of PandaArm is running due to socket reuse. Run FT publisher node separately and subscribe.

        self._ext_ft_reading = np.asarray([8675.309]*6) # Use error code as default
        # Subscribe to external FT sensor if available
        self._ext_ft_sub = rospy.Subscriber(
            '/aml/exteranal_ft/ft_sensor', WrenchStamped, self._ext_ft_callback, queue_size=1, tcp_nodelay=True)

        self._ft_smooth = {}

        # self._ft_smoother = rospy.Subscriber('/aml/ft_smoother/ft_val', FTMessage, self._ft_smooth_callback, queue_size=1, tcp_nodelay=True)

        PandaArm._configure(self, on_state_callback)

    def _ext_ft_callback(self, msg):

        self._ext_ft_reading = np.asarray([msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,
                                msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z])

    def _ft_smooth_callback(self, msg):
        
        self._ft_smooth['force']=np.asarray(msg.force)
        self._ft_smooth['torque']=np.asarray(msg.torque)

    @property
    def limb_name(self):
        return "right"

    def _update_tip_state(self, tipstate_msg):
        tip_state = {}

        PandaArm._update_tip_state(self, tipstate_msg)
        force = self._tip_state['force']
        torque = self._tip_state['torque']
        # ----- transform ft to right hand frame ('/right_hand')
        if self._transform_ft_vals:
            rotation_mat = quaternion.as_rotation_matrix(np.quaternion(ori.w, ori.x, ori.y, ori.z))
            tip_state['force'] = np.dot(rotation_mat,np.asarray([-force[0], -force[1], -force[2]]))
            tip_state['torque'] = np.dot(rotation_mat,np.asarray([-torque[0], -torque[1], -torque[2]]))            
        else:
            tip_state['force'] = np.asarray([-force[0], -force[1], -force[2]])
            tip_state['torque'] = np.asarray([-torque[0], -torque[1], -torque[2]])

        if self._smooth_ft_vals:
            try:
                tip_state['force']  = self._ft_smooth['force']
                tip_state['torque'] = self._ft_smooth['torque']

                if self._log_count != 2:
                    self._logger.info("FT Smooth topic found!")
                if log_once:
                    self._log_count = 2
            except:
                if self._log_count != 1:
                    self._logger.warning("FT Smooth topic not found... assigning raw values!")
                if log_once:
                    self._log_count = 1

        tip_state['valid'] = True

        self._tip_state.update(copy.deepcopy(tip_state))

    def set_sampling_rate(self, sampling_rate=800):
        raise self._robot_interface_exceptions.NotImplementedError("PandaArm: FrankaROSInterface does not have rate changing functionality yet.")

    def enable_force_torque_transform_to_base_frame(self, boolval=True):
        """
        Enable transformation of force vector to base frame

        :param boolval: set True to transform forces to base frame
        :type boolval: bool
        """
        self._transform_ft_vals = boolval

    def _update_state(self):

        now = rospy.Time.now()

        state = PandaArm._update_state(self)

        state['ft_reading'] = self._ext_ft_reading
 
        state['gripper_state'] = self.gripper_state()

        return state

def main():
    rospy.init_node("panda_arm_untuck")

    arm = FrankaRobot(reset_frames = False)

    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument("-u", "--untuck",
        action='store_true', default=False, help="untuck arms")
    
    args = parser.parse_args(rospy.myargv()[1:])
    untuck = args.untuck

    if untuck:
        rospy.loginfo("Untucking arms")
        arm.untuck()
        rospy.loginfo("Finished Untuck")


if __name__ == '__main__':
    main()
    
