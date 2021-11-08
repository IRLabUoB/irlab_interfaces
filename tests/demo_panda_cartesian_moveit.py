import rospy
from irlab_interfaces.franka_robot import FrankaRobot
from franka_moveit.utils import create_pose_msg

if __name__ == "__main__":
    rospy.init_node("cartesian_moveit_demo")
    robot = FrankaRobot()

    robot.untuck() # send robot to neutral pose before demo starts

    # get the associated movegroup interface object (https://www.saifsidhik.page/franka_ros_interface/DOC.html#pandamovegroupinterface)
    mvt = robot.get_movegroup_interface()

    curr_pos, curr_ori = robot.ee_pose() # get current end-effector pose (only to use the same orientation for planning)

    target_pos = [0.55, 0.3, 0.3]

    # get cartesian plan to reach specified target
    plan, _ = mvt.plan_cartesian_path([create_pose_msg(target_pos, curr_ori)])

    raw_input("Press Enter to visualise plan in RViz. Make sure Trajectory display is enabled.")

    mvt.display_trajectory(plan) # display plan in rviz

    raw_input("Press Enter to execute the plan.")

    mvt.execute_plan(plan) # execute plan
