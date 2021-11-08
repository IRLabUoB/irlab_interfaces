#!/usr/bin/env python

import PyKDL
import rospy
import numpy as np
import quaternion
from urdf_parser_py.urdf import URDF
from copy import deepcopy
from irlab_interfaces.utilities.kdl_parser import kdl_tree_from_urdf_model

class RobotKinematicsInterface(object):
    """
    Robot Kinematics with PyKDL
    """
    def __init__(self, limb, ee_frame_name, additional_segment_config=None, description=None):

        if description is None:
            self._robot_model = URDF.from_parameter_server(key='robot_description')
        else:
            self._robot_model = URDF.from_xml_file(description)

        self._kdl_tree = kdl_tree_from_urdf_model(self._robot_model)

        if additional_segment_config is not None:
            for c in additional_segment_config:
                child_name = c["child_name"]
                kdl_origin = np.zeros([4,3])
                q = quaternion.from_rotation_matrix(c["origin_ori"]).tolist()
                kdl_origin_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(q.x,q.y,q.z,q.w),
                                            PyKDL.Vector(*(c["origin_pos"].tolist())))
                kdl_sgm = PyKDL.Segment(c["child_name"], PyKDL.Joint(c["joint_name"]),
                                    kdl_origin_frame, PyKDL.RigidBodyInertia())
                self._kdl_tree.addSegment(
                    kdl_sgm, c["parent_name"])
        
        self._base_link = self._robot_model.get_root()
        self._tip_link = ee_frame_name
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)

        self._arm_interface = limb
        self._joint_names = deepcopy(self._arm_interface.joint_names())
        self._num_jnts = len(self._joint_names)

        # KDL Solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain,
                                                   self._fk_p_kdl,
                                                   self._ik_v_kdl)
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._arm_chain)
        self._dyn_kdl = PyKDL.ChainDynParam(self._arm_chain,
                                            PyKDL.Vector.Zero())

    def print_robot_description(self):
        nf_joints = 0
        for j in self._robot_model.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print("URDF non-fixed joints: %d;" % nf_joints)
        print("URDF total joints: %d" % len(self._robot_model.joints))
        print("URDF links: %d" % len(self._robot_model.links), [link.name for link in self._robot_model.links])
        print("KDL joints: %d" % self._kdl_tree.getNrOfJoints())
        print("KDL segments: %d" % self._kdl_tree.getNrOfSegments())

    def print_kdl_chain(self):
        for idx in range(self._arm_chain.getNrOfSegments()):
            print('* ' + self._arm_chain.getSegment(idx).getName())

    def joints_to_kdl(self, type, values=None):
        kdl_array = PyKDL.JntArray(self._num_jnts)
        pos_array = PyKDL.JntArray(self._num_jnts)

        if values is None:
            if type == 'positions':
                cur_type_values = self._arm_interface.joint_angles()
            elif type == 'velocities':
                cur_type_values = self._arm_interface.joint_velocities()
                pos_list = self._arm_interface.joint_angles()

            elif type == 'torques':
                cur_type_values = self._arm_interface.joint_efforts()
        else:
            cur_type_values = values
        for idx, name in enumerate(self._joint_names):
            kdl_array[idx] = cur_type_values[name]
            if type == 'velocities':
                pos_array[idx] = pos_list[name]

        if type == 'velocities':
            kdl_array = PyKDL.JntArrayVel(pos_array, kdl_array) # ----- using different constructor for getting velocity fk
        return kdl_array

    def kdl_to_mat(self, data):
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat

    def forward_position_kinematics(self,joint_values=None):
        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(self.joints_to_kdl('positions',joint_values),
                                 end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def forward_velocity_kinematics(self,joint_velocities=None):
        end_frame = PyKDL.FrameVel()
        self._fk_v_kdl.JntToCart(self.joints_to_kdl('velocities',joint_velocities),
                                 end_frame)
        # print 
        return end_frame.GetTwist()

    def inverse_kinematics(self, position, orientation=None, seed=None):
        ik = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation is not None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        if orientation is not None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_jnts)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            return result
        else:
            return None

    def jacobian(self,joint_values=None):
        jacobian = PyKDL.Jacobian(self._num_jnts)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self.kdl_to_mat(jacobian)

    def jacobian_transpose(self,joint_values=None):
        return self.jacobian(joint_values).T

    def jacobian_pseudo_inverse(self,joint_values=None):
        return np.linalg.pinv(self.jacobian(joint_values))


    def inertia(self,joint_values=None):
        inertia = PyKDL.JntSpaceInertiaMatrix(self._num_jnts)
        self._dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self.kdl_to_mat(inertia)

    def cart_inertia(self,joint_values=None):
        js_inertia = self.inertia(joint_values)
        jacobian = self.jacobian(joint_values)
        return np.linalg.inv(jacobian * np.linalg.inv(js_inertia) * jacobian.T)


if __name__ == '__main__':
    
    rospy.init_node('test')
    from aml_robot.panda_robot import PandaArm
    r = PandaArm()
    # from aml_robot.sawyer_robot import SawyerArm
    # r = SawyerArm()
    # print r.has_gripper
    kin = r._kinematics



    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # argument = r.joint_angles()
        # combine the names and joint angles to a dictionary, that only is accepted by kdl
        # jacobian = np.array(kin.jacobian(argument))

        print("-----------------")
        print("")
        # print jacobian
        # print (kin.forward_position_kinematics())
        print(kin.forward_velocity_kinematics())
        print("")

        print(r._cartesian_velocity)
        # print (r.ee_pose())
        print("-----------------")
        print("-----------------")
        print("-----------------")

        # break
        rate.sleep()
