from __future__ import absolute_import
import tf
import numpy as np
import quaternion
from tf import TransformListener

def convert_rospy_time2sec(rospy_time):
    
    return rospy_time.secs + rospy_time.nsecs*1e-9


def convert_pose_to_euler_tranform(pose):
    '''
    expects a pose vector that is 7 dimensional in the following format
    pose = np.array([x,y,z, quat_x, quat_y, quat_z, quat_w])

    '''

    if len(pose) != 7:
        print("Incorrect pose format")
        raise ValueError


    rot = quaternion.as_rotation_matrix(np.quaternion(pose[6], pose[3], pose[4], pose[5]))

    return np.vstack([np.hstack([rot, np.array([[pose[0]], [pose[1]], [pose[2]]])]),np.array([0.,0.,0.,1.])])



def get_transform(base_frame, source_frame):

    tf = TransformListener()

    if base_frame == source_frame:
        return (0, 0, 0), (0, 0, 0, 1), rospy.Time.now()

    # t = (0,0,0)
    # q = (0,0,0,1)
    time = None

    while time is None:
        try:

            time = tf.getLatestCommonTime(source_frame, base_frame)
        except:
            rospy.loginfo("Failed to get common time between %s and %s. Trying again..."%(source_frame,base_frame,))

    t = None
    q = None
    try:
        t, q = tf.lookupTransform(base_frame, source_frame, time)#self._tf_buffer.lookup_transform(frame_name, base_frame, rospy.Time(0), rospy.Duration(5.0))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Transform could not be queried.")

        return (0, 0, 0), (0, 0, 0, 1), rospy.Time.now()

    translation = (t[0], t[1], t[2])
    quaternion = (q[0], q[1], q[2], q[3])
    # translation = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
    # quaternion = (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)

    return translation, quaternion, time

def to_transform_matrix(t, q):

    t_mat = tf.transformations.translation_matrix(t)
    r_mat = tf.transformations.quaternion_matrix(q)
    transform_mat = np.dot(t_mat, r_mat)

    return transform_mat


