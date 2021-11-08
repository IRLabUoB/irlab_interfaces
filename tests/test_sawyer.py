
import geometry_msgs.msg
import quaternion
import rospy
from irlab_interfaces.sawyer_robot import SawyerRobot

from geometry_msgs.msg import Pose

poses = [
         [-0.0009228515625, -1.17875390625, -0.0023515625, 2.1775595703125, 0.003744140625, 0.57066796875, 3.314966796875],
         [-0.3503349609375, -1.002400390625, -0.4102822265625, 1.4559833984375, -0.895927734375, 0.837537109375, 3.16163671875]
        ]

def create_pose_msg(ee_pose):
    pose = Pose()
    pose.position.x = ee_pose[0][0]
    pose.position.y = ee_pose[0][1]
    pose.position.z = ee_pose[0][2]

    pose.orientation.x = ee_pose[1].x
    pose.orientation.y = ee_pose[1].y
    pose.orientation.z = ee_pose[1].z
    pose.orientation.w = ee_pose[1].w

    return pose

def change_ori_pose_msg(msg, quat):
    msg.orientation.x = quat.x
    msg.orientation.y = quat.y
    msg.orientation.z = quat.z
    msg.orientation.w = quat.w

    return msg

def new_ori(quat1, quat2):

    return quaternion.from_rotation_matrix(quaternion.as_rotation_matrix(quat1)*quaternion.as_rotation_matrix(quat2))

def create_pose_stamped_msg(position, orientation, frame = "base"):

    pose = geometry_msgs.msg.PoseStamped()

    pose.header.frame_id = frame

    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]

    if isinstance(orientation,quaternion.quaternion):
        pose.pose.orientation.x = orientation.x
        pose.pose.orientation.y = orientation.y
        pose.pose.orientation.z = orientation.z
        pose.pose.orientation.w = orientation.w
    else:
        pose.pose.orientation.x = orientation[1]
        pose.pose.orientation.y = orientation[2]
        pose.pose.orientation.z = orientation[3]
        pose.pose.orientation.w = orientation[0]

    return pose

IRLab_workspace = [
           {
           'name': 'back_wall',
           'pose': create_pose_stamped_msg(position = [-0.57,0.0,0.5], orientation = [1,0,0,0]),
           'size': [0.1,1.8,1]
           },
           {
           'name': 'side_wall',
           'pose': create_pose_stamped_msg(position = [-0.3,-0.85,0.5], orientation = [1,0,0,0]),
           'size': [0.6,0.1,1]
           },
           {
           'name': 'table',
           'pose': create_pose_stamped_msg(position = [0.45,-0.0,0], orientation = [1,0,0,0]),
           'size': [2,1.8,0.02]
           },
           {
           'name': 'controller_box',
           'pose': create_pose_stamped_msg(position = [-0.37,0.55,0.08], orientation = [1,0,0,0]),
           'size': [0.4,0.6,0.16]
           },
           {
           'name': 'equipment_box',
           'pose': create_pose_stamped_msg(position = [-0.35,-0.68,0.17], orientation = [1,0,0,0]),
           'size': [0.46,0.4,0.34]
           }
            ]

if __name__ == '__main__':

    rospy.init_node("panda_env")
    
    r = SawyerRobot() # handle to use methods from SawyerRobot class
    
    kin = r._kinematics # to test the kinematics (not required, can directly query kinematics using  methods in SawyerRobot)

    # neutral = r.move_to_neutral 
    # move_to = r.move_to_joint_position

    cpm = create_pose_msg
    # v1 = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.]
    # v2 = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
    # v3 = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
    # v4 = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

    # coll = r._collision_behaviour_interface
    # In interactive mode, for instance enter 
    #               $ neutral()
    #   to make the robot move to neutral pose
    # or type $ move_to(poses[0]) to move to the first joint pose from the list defined above (make sure robot workspace is free)

    mvt = r.get_movegroup_interface()

    if mvt is not None:
        move_to = mvt.go_to_joint_positions
        rospy.sleep(1) # ----- Not having this delay sometimes caused failing to create some boxes

        for config in IRLab_workspace:

            rospy.loginfo("-- Creating object: {}..".format(config['name']))
            success = mvt._scene.add_box(**config)
            rospy.loginfo("------ {}".format("success" if success else "FAILED!"))

        rospy.loginfo("Created Demo Planning Scene.")
    else:
        move_to = r.move_to_joint_position

    # q45x = quaternion.quaternion(0.924,0.383,0.,0.)
    # q315x = quaternion.quaternion(0.924,-0.383,0.,0.)

    # rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     print r.get_robot_status()['robot_mode']
    #     print ''
    #     rate.sleep()


