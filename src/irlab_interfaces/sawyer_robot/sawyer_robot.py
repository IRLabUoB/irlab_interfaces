#!/usr/bin/env python
from __future__ import absolute_import
# General ROS imports
import rospy

from std_msgs.msg import (
    UInt16,
)
import copy
# Auxiliary imports
import numpy as np
import quaternion

# Intera SDK imports
import intera_interface
import argparse
# from intera_interface import CHECK_VERSION
CHECK_VERSION = False
from intera_core_msgs.msg import SEAJointState, EndpointStates

# Robot specific imports
from irlab_interfaces.sawyer_robot.sawyer_ik import IKSawyer

# additional imports
from irlab_interfaces.irlab_perception import camera_sensor
from irlab_interfaces.utilities.quaternion_utils import compute_omg  # for computing orientation error
from irlab_interfaces.robot_interface import RobotInterface
from irlab_interfaces.utilities import RobotMoveGroupInterface, RobotKinematicsInterface
# from intera_motion_interface import InteractionOptions
# from intera_core_msgs.msg import InteractionControlCommand

from aml_io.log_utils import aml_logging

log_once = True

class SawyerRobot(intera_interface.Limb, RobotInterface):
    def __init__(self, limb="right", on_state_callback=None):

        RobotInterface.__init__(self)

        # these values are from the baxter urdf file
        self._jnt_limits = [{'lower': -1.70167993878, 'upper': 1.70167993878},
                            {'lower': -2.147, 'upper': 1.047},
                            {'lower': -3.05417993878, 'upper': 3.05417993878},
                            {'lower': -0.05, 'upper': 2.618},
                            {'lower': -3.059, 'upper': 3.059},
                            {'lower': -1.57079632679, 'upper': 2.094},
                            {'lower': -3.059, 'upper': 3.059}]

        # number of joints
        self._nq = len(self._jnt_limits)
        # number of control commads
        self._nu = len(self._jnt_limits)

        self._ready = False

        self._limb = limb

        self._log_count = 0

        self._configure(limb, on_state_callback)

        self._ready = True

        self._q_mean = np.array([0.5 * (limit['lower'] + limit['upper']) for limit in self._jnt_limits])

        if self._gripper is not None:
            self._jnt_limits.append({'lower': self._gripper.MIN_POSITION, 'upper': self._gripper.MAX_POSITION})

        if self._gripper is not None:
            self._tuck = np.array(
                [-3.31223050e-04, -1.18001699e+00, -8.22146399e-05, 2.17995802e+00, -2.70787321e-03, 5.69996851e-01,
                 3.32346747e+00])#, 2.07798000e-02]) 
        else:
            self._tuck = np.array(
                [-3.31223050e-04, -1.18001699e+00, -8.22146399e-05, 2.17995802e+00, -2.70787321e-03, 5.69996851e-01,
                 3.32346747e+00]) 

        self._untuck = self._tuck

        self._intera_robot_enable_interface = intera_interface.RobotEnable(CHECK_VERSION)
        self._intera_robot_enable_interface.enable()

        # this will be useful to compute ee_velocity using finite differences
        self._ee_pos_old, self._ee_ori_old = self.ee_pose()
        self._time_now_old = self.time_in_seconds()

        self._count = 0

        try:
            self._movegroup_interface = RobotMoveGroupInterface("right_arm", "right_hand")
        except:
            rospy.loginfo("MoveGroup was not found! This is okay if moveit service is not required!")
            self._movegroup_interface = None

        update_check = rospy.Duration(1.0)

        # rospy.Timer(update_check, self.check)

    @property
    def limb_name(self):
        return self._limb

    def get_movegroup_interface(self):
        return self._movegroup_interface

    def check(self, event):

        if not rospy.is_shutdown():

            print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
            print(("Count is \t", self._count))
            print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
            self._count = 0

    def enable_robot(self):
        self._intera_robot_enable_interface.enable()

    def disable_robot(self):
        self._intera_robot_enable_interface.disable()

    def _configure_cuff(self):

        self._has_cuff = True

        try:
            self._cuff_state = None

            self._cuff = intera_interface.Cuff(limb=self._limb)
            # connect callback fns to signals
            self._lights = None
            self._lights = intera_interface.Lights()
            self._cuff.register_callback(self._light_action, '{0}_cuff'.format(self._limb))

        except Exception as e:
            print(e)
            self._logger.warning(e)
            self._has_cuff = False

    def _configure_gripper(self):

        try:
            self._gripper = intera_interface.Gripper('{0}_gripper'.format(self._limb))
            if not (self._gripper.is_calibrated() or self._gripper.calibrate() == True):
                self._logger.error("({0}_gripper) calibration failed.".format(self._gripper.name))
                raise

            if self._has_cuff:
                self._cuff.register_callback(self._close_action, '{0}_button_upper'.format(self._limb))
                self._cuff.register_callback(self._open_action, '{0}_button_lower'.format(self._limb))

            self._logger.info("{0} Cuff Control initialized...".format(self._gripper.name))

        except Exception as e:
            self._gripper = None

    def _open_action(self, value):
        if value and self._gripper.is_ready():
            self._logger.debug("gripper open triggered")
            self._gripper.open()
            if self._lights:
                self._set_lights('red', False)
                self._set_lights('green', True)

    def _close_action(self, value):
        if value and self._gripper.is_ready():
            self._logger.debug("gripper close triggered")
            self._gripper.close()
            if self._lights:
                self._set_lights('green', False)
                self._set_lights('red', True)

    def _light_action(self, value):
        if value:
            self._logger.debug("cuff grasp triggered")
        else:
            self._logger.debug("cuff release triggered")
        if self._lights:
            self._set_lights('red', False)
            self._set_lights('green', False)
            self._set_lights('blue', value)

    def _set_lights(self, color, value):
        self._lights.set_light_state('head_{0}_light'.format(color), on=bool(value))
        self._lights.set_light_state('{0}_hand_{1}_light'.format(self._limb, color), on=bool(value))

    def cuff_cb(self, value):
        if self._has_cuff:
            self._cuff_state = value
        else:
            self._logger.warning("CUFF NOT DETECTED")


    @property
    def get_lfd_status(self):
        """
        this function returns self._cuff_state to be true
        when arm is moved by a demonstrator, the moment arm stops
        moving, the status returns to false
        initial value of the cuff is None, it is made False by pressing the
        cuff button once
        """
        if self._has_cuff:
            return self._cuff.cuff_button()
        else:
            self._logger.warning("CUFF NOT DETECTED")
            return None

    def set_sampling_rate(self, sampling_rate=800):
        self._pub_rate.publish(sampling_rate)

    def tuck(self):
        self._logger.warning("NOT IMPLEMENTED")

    def untuck(self):
        self.move_to_neutral()

    def _configure(self, limb, on_state_callback):
        self._state = None

        if on_state_callback:
            self._on_state_callback = on_state_callback
        else:
            self._on_state_callback = lambda m: None

        # Parent constructor
        intera_interface.Limb.__init__(self, limb)

        self._kinematics = RobotKinematicsInterface(self, self.name + '_hand')

        self._ik_sawyer = IKSawyer(limb=self)

        self._ik_sawyer.configure_ik_service()

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=1)

        # gravity + feed forward torques
        self._h = [0. for _ in range(self._nq)]

        self.set_sampling_rate()

        self.set_command_timeout(0.2)

        self._camera = camera_sensor.CameraSensor()

        self._ft = None
        self._transform_ft_vals = True # transform forces from FT sensor frame to base frame
        self._smooth_ft_vals = True
        try:

            self._ft = ft_sensor.FTSensor(ip=get_ip(),imu="use_mock")
        except:
            pass

        self._gripper = None
        self._cuff = None
        self._tip_state = {}
        
        self._gravity_comp_data = {}

        # self._ft_smooth = {}

        # self._ft_smoother = rospy.Subscriber('/aml/ft_smoother/ft_val', FTMessage, self._ft_smooth_callback, queue_size=1, tcp_nodelay=True)

        self._gravity_comp = rospy.Subscriber('robot/limb/' + limb + '/gravity_compensation_torques',
                                              SEAJointState, self._gravity_comp_callback, queue_size=1, tcp_nodelay=True)

        self._tip_states = rospy.Subscriber('robot/limb/' + limb + '/tip_states',
                                              EndpointStates, self._tip_states_callback, queue_size=1, tcp_nodelay=True) 


        self._configure_cuff()
        # self._configure_gripper()

    # def _ft_smooth_callback(self, msg):
        
        # self._ft_smooth['force']=np.asarray(msg.force)
        # self._ft_smooth['torque']=np.asarray(msg.torque)


    def _gravity_comp_callback(self, msg):

        self._h = msg.gravity_model_effort

        self._gravity_comp_data['cmd_position'] = np.asarray(msg.commanded_position)
        self._gravity_comp_data['cmd_velocity'] = np.asarray(msg.commanded_velocity)
        self._gravity_comp_data['cmd_acceleration'] = np.asarray(msg.commanded_acceleration)
        self._gravity_comp_data['cmd_effort'] = np.asarray(msg.commanded_effort)
        self._gravity_comp_data['actual_position'] = np.asarray(msg.actual_position)
        self._gravity_comp_data['actual_velocity'] = np.asarray(msg.actual_velocity)
        self._gravity_comp_data['actual_effort'] = np.asarray(msg.actual_effort)
        self._gravity_comp_data['gravity_model_effort'] = np.asarray(msg.gravity_model_effort)
        self._gravity_comp_data['gravity_only'] = np.asarray(msg.gravity_only)
        self._gravity_comp_data['interaction_torque'] = np.asarray(msg.interaction_torque)

    def _tip_states_callback(self, msg):
        tip_state = {}
        
        time = msg.header.stamp
        pos = msg.states[0].pose.position
        ori = msg.states[0].pose.orientation
        l_vel = msg.states[0].twist.linear
        a_vel = msg.states[0].twist.angular
        force = msg.states[0].wrench.force # ----- the force appears to be in a left-hand coordinate frame !!!!!!!!!!!!!!! WTF?!
        torque = msg.states[0].wrench.torque

        tip_state['time'] = {'secs':time.secs, 'nsecs':time.nsecs}
        tip_state['position'] = np.asarray([pos.x, pos.y, pos.z])
        tip_state['orientation'] = np.asarray([ori.w, ori.x, ori.y, ori.z])
        tip_state['linear_vel'] = np.asarray([l_vel.x, l_vel.y, l_vel.z])
        tip_state['angular_vel'] = np.asarray([a_vel.x, a_vel.y, a_vel.z])

        # ----- transform ft to right hand frame ('/right_hand')

        if self._transform_ft_vals and not self._smooth_ft_vals:
            rotation_mat = quaternion.as_rotation_matrix(np.quaternion(ori.w, ori.x, ori.y, ori.z))
            tip_state['force'] = np.dot(rotation_mat,np.asarray([-force.x, -force.y, -force.z]))
            tip_state['torque'] = np.dot(rotation_mat,np.asarray([-torque.x, -torque.y, -torque.z]))
        elif self._transform_ft_vals and self._smooth_ft_vals:
            # try:
            #     tip_state['force']  = self._ft_smooth['force']
            #     tip_state['torque'] = self._ft_smooth['torque']

            #     if self._log_count != 2:
            #         self._logger.info("FT Smooth topic found!")
            #     if log_once:
            #         self._log_count = 2
            # except:
            #     if self._log_count != 1:
            #         self._logger.warning("FT Smooth topic not found... assigning zeros!")
            #     if log_once:
            #         self._log_count = 1
            tip_state['force']  = np.zeros(3)
            tip_state['torque'] = np.zeros(3)
            
        else:
            tip_state['force'] = np.asarray([-force.x, -force.y, -force.z])
            tip_state['torque'] = np.asarray([-torque.x, -torque.y, -torque.z])
        tip_state['valid'] = msg.states[0].valid

        self._tip_state = tip_state

    def enable_force_torque_transform_to_base_frame(self, boolval=True):
        self._transform_ft_vals = boolval
        

    def _update_state(self):

        # self._count += 1

        now = rospy.Time.now()

        joint_angles = self.angles()
        joint_velocities = self.velocities()
        joint_efforts = self.joint_efforts()
        tip_state = self.tip_state()

        joint_names = self.joint_names()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        state = {}
        state['position'] = joint_angles
        state['velocity'] = joint_velocities
        state['effort'] = np.array(to_list(joint_efforts))
        state['jacobian'] = self.jacobian(None)
        state['inertia'] = self.inertia(None)
        state['rgb_image'] = self._camera._curr_rgb_image
        state['depth_image'] = self._camera._curr_depth_image
        state['gravity_comp'] = self._gravity_comp_data#np.array(self._h)
        state['tip_state'] = tip_state

        state['timestamp'] = {'secs': now.secs, 'nsecs': now.nsecs}
        try:
            state['ee_point'], state['ee_ori'] = self.ee_pose()
        except:
            pass

        try:
            tmp = state['jacobian'].dot(state['velocity'])
            state['ee_vel'], state['ee_omg'] = tmp[:3], tmp[3:]#self.ee_velocity()
        except:
            pass

        try:
            if self._transform_ft_vals:
                rotation_mat = quaternion.as_rotation_matrix(state['ee_ori'])
                state['ft_reading'] = self._ft.ft_reading(rotation_mat = rotation_mat)
            else:
                state['ft_reading'] = self._ft.ft_reading()
 
        except:
            state['ft_reading'] = None

        state['gripper_state'] = self.gripper_state()

        # ee_velocity              = self.endpoint_velocity()['linear']
        # state['ee_velocity']     = np.array([ee_velocity.x, ee_velocity.y, ee_velocity.z])

        return state

    def angles(self, include_gripper=False):
        joint_angles = self.joint_angles()

        joint_names = self.joint_names()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        all_angles = to_list(joint_angles)

        if include_gripper and self._gripper is not None:
            all_angles.append(self._gripper.get_position())
        return np.array(all_angles)

    def joint_limits(self):

        return self._jnt_limits

    def velocities(self, include_gripper=False):
        joint_velocities = self.joint_velocities()

        joint_names = self.joint_names()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        all_velocities = to_list(joint_velocities)

        if include_gripper and self._gripper is not None:
            all_velocities.append(0.0)
        return np.array(all_velocities)

    def q_mean(self):
        return self._q_mean

    def n_cmd(self):
        return self._nu

    def n_joints(self):
        return self._nq

    def state(self):
        return self._state

    def tip_state(self):
        return self._tip_state

    def gripper_state(self):
        gripper_state = {}

        if self._gripper is not None:
            gripper_state['position'] = self._gripper.get_position()
            gripper_state['force'] = self._gripper.get_force()

        return gripper_state

    def set_gripper_speed(self, speed):

        if self._gripper is not None:
            self._gripper.set_velocity(speed)

    def set_arm_speed(self, speed):

        self.set_joint_position_speed(speed)

    def _on_joint_states(self, msg):
        intera_interface.Limb._on_joint_states(self, msg)

        if self._ready:
            self._state = self._update_state()
            self._on_state_callback(self._state)

    def end_effector_link_name(self):
        return self._kinematics._tip_link

    def base_link_name(self):
        return self._kinematics._base_link

    def exec_gripper_cmd(self, pos, force=None):

        if self._gripper is None:
            return

        if force is not None:
            holding_force = min(max(self._gripper.MIN_FORCE, force), self._gripper.MAX_FORCE)

            self._gripper.set_holding_force(holding_force)

        position = min(self._gripper.MAX_POSITION, max(self._gripper.MIN_POSITION, pos))

        self._gripper.set_position(pos)

    def exec_gripper_cmd_delta(self, pos_delta, force_delta=None):

        if self._gripper is None:
            return

        if force_delta is not None:
            force = self._gripper.get_force()
            holding_force = min(max(self._gripper.MIN_FORCE, force + force_delta), self._gripper.MAX_FORCE)

            self._gripper.set_holding_force(holding_force)

        pos = self._gripper.get_position()
        position = min(self._gripper.MAX_POSITION, max(self._gripper.MIN_POSITION, pos + pos_delta))

        self._gripper.set_position(position)

    def exec_position_cmd(self, cmd):
        # there is some issue with this function ... move_to_joint_pos works far better.

        curr_q = self._state['position']

        if len(cmd) > 7:
            gripper_cmd = cmd[7:]
            self.exec_gripper_cmd(*gripper_cmd)

        joint_command = dict(list(zip(self.joint_names(), cmd[:7])))

        self.set_joint_positions(joint_command)

    def exec_position_cmd_delta(self, cmd):
        curr_q = self.joint_angles()
        joint_names = self.joint_names()

        joint_command = dict([(joint, curr_q[joint] + cmd[i]) for i, joint in enumerate(joint_names)])
        self.set_joint_positions(joint_command)

        if len(cmd) > 7:
            gripper_cmd = cmd[7:]
            self.exec_gripper_cmd_delta(*gripper_cmd)

    def move_to_joint_pos_delta(self, cmd):
        curr_q = self.joint_angles()
        joint_names = self.joint_names()

        joint_command = dict([(joint, curr_q[joint] + cmd[i]) for i, joint in enumerate(joint_names)])

        self.move_to_joint_positions(joint_command)

    def exec_velocity_cmd(self, cmd):

        joint_names = self.joint_names()

        # can't we use dict(zip(joint_names, cmd)) to combine two lists?
        velocity_command = dict([(joint, cmd[i]) for i, joint in enumerate(joint_names)])

        self.set_joint_velocities(velocity_command)

    def exec_torque_cmd(self, cmd):

        joint_names = self.joint_names()

        torque_command = dict([(joint, cmd[i]) for i, joint in enumerate(joint_names)])

        self.set_joint_torques(torque_command)

    def move_to_joint_position(self, joint_angles):
        self.move_to_joint_positions(dict(list(zip(self.joint_names(), joint_angles))))

    def ee_pose(self):

        ee_point = self.endpoint_pose()['position']
        ee_point = np.array([ee_point.x, ee_point.y, ee_point.z])

        ee_ori = self.endpoint_pose()['orientation']
        ee_ori = np.quaternion(ee_ori.w, ee_ori.x, ee_ori.y, ee_ori.z)

        return ee_point, ee_ori

    def time_in_seconds(self):
        time_now = rospy.Time.now()
        return time_now.secs + time_now.nsecs * 1e-9

    def ee_velocity(self, real_robot=True):
        # this is a simple finite difference based velocity computation
        # please note that this might produce a bug since self._goal_ori_old gets
        # updated only if get_ee_vel is called.
        # TODO : to update in get_ee_pose or find a better way to compute velocity

        if real_robot:

            ee_velocity = self.endpoint_velocity()['linear']
            ee_vel = np.array([ee_velocity.x, ee_velocity.y, ee_velocity.z])
            ee_omega = self.endpoint_velocity()['angular']
            ee_omg = np.array([ee_omega.x, ee_omega.y, ee_omega.z])

        else:

            time_now_new = self.time_in_seconds()

            ee_pos_new, ee_ori_new = self.ee_pose()

            dt = time_now_new - self._time_now_old

            ee_vel = (ee_pos_new - self._ee_pos_old) / dt

            ee_omg = compute_omg(ee_ori_new, self._ee_ori_old) / dt

            self._goal_ori_old = ee_ori_new
            self._goal_pos_old = ee_pos_new
            self._time_now_old = time_now_new

        return ee_vel, ee_omg

    def forward_kinematics(self, joint_angles=None, ori_type='quat'):

        if joint_angles is None:

            argument = None

        else:

            argument = dict(list(zip(self.joint_names(), joint_angles)))

        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        pose = np.array(self._kinematics.forward_position_kinematics(argument))
        position = pose[0:3][:, None]  # senting as  column vector

        w = pose[6]
        x = pose[3]
        y = pose[4]
        z = pose[5]  # quarternions

        rotation = quaternion.quaternion(w, x, y, z)

        # formula for converting quarternion to rotation matrix

        if ori_type == 'mat':

            # rotation = np.array([[1.-2.*(y**2+z**2),    2.*(x*y-z*w),           2.*(x*z+y*w)],\
            #                      [2.*(x*y+z*w),         1.-2.*(x**2+z**2),      2.*(y*z-x*w)],\
            #                      [2.*(x*z-y*w),         2.*(y*z+x*w),           1.-2.*(x**2+y**2)]])

            rotation = quaternion.as_rotation_matrix(rotation)

        elif ori_type == 'eul':

            rotation = quaternion.as_euler_angles(rotation)
        elif ori_type == 'quat':
            pass

        return position, rotation

    def cartesian_velocity(self, joint_angles=None):

        if joint_angles is None:

            argument = None

        else:

            argument = dict(list(zip(self.joint_names(), joint_angles)))

        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        return np.array(self._kinematics.forward_velocity_kinematics(argument))[0:3]  # only position

    def jacobian(self, joint_angles=None):

        if joint_angles is None:

            argument = None

        else:

            argument = dict(list(zip(self.joint_names(), joint_angles)))
        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        jacobian = np.array(self._kinematics.jacobian(argument))

        return jacobian

    def inertia(self, joint_angles=None):
        if joint_angles is None:
            argument = None
        else:
            argument = dict(list(zip(self.joint_names(), joint_angles)))

        return np.array(self._kinematics.inertia(argument))

    def inverse_kinematics(self, pos, ori=None, seed=None, null_space_goal = None, use_service=False):
        success = False
        soln = None

        if ori is not None:
            #expects a pykdl quaternion which is of type x,y,z,w 
            if isinstance(ori, np.quaternion):
                ori = np.array([ori.x, ori.y, ori.z, ori.w])

        if use_service:
            success, soln = self._ik_sawyer.ik_service_request(pos=pos, ori=ori, seed_angles=seed, null_space_goal=null_space_goal, use_advanced_options=True)
        else:
            soln = self._kinematics.inverse_kinematics(position=pos, orientation=ori, seed=seed)

            if soln is not None:
                success = True

        return success, soln


def main():
    rospy.init_node("sawyer_arm_untuck")

    arm = SawyerRobot('right')

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

    #     print "this",arm.state()['ft_reading'][:3]
    #     # print arm.state()['ee_ori']
    #     # arm.state()
    #     # pass
    #     rospy.sleep(0.1)

    # rospy.spin()


if __name__ == '__main__':
    main()
