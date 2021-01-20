import logging
from prpy.exceptions import PrPyException
import adapy.controller_client

logger = logging.getLogger('adapy.util')


class AdaPyException(PrPyException):
    pass


def find_adapy_resource(relative_path, package='adapy'):
    """ Returns the full path to a desired file given a ROS package, and the relative path
    of that file in the package.

    @param relative_path: The path and filename relative to the package path
    @type relative_path: str
    @param package: Name of the ROS package
    @type package: str
    @return Full path and filename
    @rtype str
    """
    from catkin.find_in_workspaces import find_in_workspaces

    paths = find_in_workspaces(project=package, search_dirs=['share'],
                               path=relative_path, first_match_only=True)

    if paths and len(paths) == 1:
        return paths[0]
    else:
        raise IOError('Loading AdaPy resource "{:s}" failed.'.format(
                      relative_path))


def or_to_ros_trajectory(robot, traj, time_tolerance=0.01):
    """ Convert an OpenRAVE trajectory to a ROS trajectory.

    @param robot: OpenRAVE robot
    @type  robot: openravepy.Robot
    @param traj: input trajectory
    @type  traj: openravepy.Trajectory
    @param time_tolerance: minimum time between two waypoints
    @type  time_tolerance: float
    """
    import numpy
    from rospy import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    assert time_tolerance >= 0.

    if traj.GetEnv() != robot.GetEnv():
        raise ValueError(
            'Robot and trajectory are not in the same environment.')

    cspec = traj.GetConfigurationSpecification()
    dof_indices, _ = cspec.ExtractUsedIndices(robot)

    traj_msg = JointTrajectory(
        joint_names=[ robot.GetJointFromDOFIndex(dof_index).GetName()
                      for dof_index in dof_indices ]
    )

    time_from_start = 0.
    prev_time_from_start = 0.
    
    
    for iwaypoint in xrange(traj.GetNumWaypoints()):
        waypoint = traj.GetWaypoint(iwaypoint)

        dt = cspec.ExtractDeltaTime(waypoint)
        q = cspec.ExtractJointValues(waypoint, robot, dof_indices, 0)
        qd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 1)
        qdd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 2)

        if dt is None:
            raise ValueError('Trajectory is not timed.')
        elif q is None:
            raise ValueError('Trajectory does not contain joint values')
        elif qdd is not None and qd is None:
            raise ValueError('Trajectory contains accelerations,'
                             ' but not velocities.')

        # Duplicate waypoints break trajectory execution, so we explicitly
        # filter them out. Note that we check the difference in time between
        # the current and the previous waypoint, not the raw "dt" value. This
        # is necessary to support very densely sampled trajectories.
        time_from_start += dt
        deltatime = time_from_start - prev_time_from_start

        # openrave includes the first trajectory point as current, with time zero
        # ros ignores this. so if time zero, then skip
        if time_from_start == 0:
          continue

        if iwaypoint > 0 and deltatime < time_tolerance:
            logger.warning('Skipped waypoint %d because deltatime is %.3f < %.3f.',
                iwaypoint, deltatime, time_tolerance)
            continue

        prev_time_from_start = time_from_start

        # Create the waypoint.
        traj_msg.points.append(
            JointTrajectoryPoint(
                positions=list(q),
                velocities=list(qd) if qd is not None else [],
                accelerations=list(qdd) if qdd is not None else [],
                time_from_start=Duration.from_sec(time_from_start)
            )
        )

    assert abs(time_from_start - traj.GetDuration()) < time_tolerance
    return traj_msg


def pad_ros_trajectory(robot, traj_ros, joint_names):
    """ Add constant values for DOFs missing from a ROS trajectory.

    @param robot: OpenRAVE robot
    @type  robot: openravepy.Robot
    @param traj_ros: ROS trajectory message
    @type  traj_ros: trajectory_msgs.msg.JointTrajectory
    @param joint_names: List of joint names to ensure are present
    @type  joint_names: [str]
    """
    missing_names = set(joint_names).difference(traj_ros.joint_names)
    missing_values = [
        robot.GetJoint(joint_name).GetValue(0)
        for joint_name in missing_names
    ]
    zero_values = [0.0] * len(missing_values)

    for waypoint in traj_ros.points:
        if len(waypoint.positions) > 0:
            waypoint.positions.extend(missing_values)
        if len(waypoint.velocities) > 0:
            waypoint.velocities.extend(zero_values)
        if len(waypoint.accelerations) > 0:
            waypoint.accelerations.extend(zero_values)

    traj_ros.joint_names.extend(missing_names)
    return traj_ros



def publish_joint_state_once(robot, pub=None, topic='/joint_states'):
    import rospy
    import sensor_msgs.msg

    if not robot.simulated:
        raise RuntimeError('Refusing to publish joints of a real robot')
    if pub is None:
        pub = rospy.Publisher(topic, sensor_msgs.msg.JointState, queue_size=1)
    msg = sensor_msgs.msg.JointState()
    msg.header.stamp = rospy.get_rostime()
    msg.name = [ j.GetName() for j in robot.GetJoints() ]
    msg.position = robot.GetDOFValues()
    msg.velocity = robot.GetDOFVelocities()
    msg.effort = [ 0. ] * len(msg.name)
    pub.publish(msg)

def publish_joint_states(robot, topic='/joint_states', rate=50.):
    import rospy
    import sensor_msgs.msg
    pub = rospy.Publisher(topic, sensor_msgs.msg.JointState, queue_size=1)
    return rospy.Timer(rospy.Duration(1./rate, ), lambda _: publish_joint_state_once(robot, pub))   



class ControllerPauser(object):
    def __init__(self, mode_switcher):
        """ Construct a context manager for stopping all current controllers.

        @param mode_switcher: mode switcher object
        @type  mode_switcher: ModeSwitcher
        """
        self._mode_switcher = mode_switcher
        self._stopped_controllers = None

    def __enter__(self):
        self._stopped_controllers = self.switch()

    def __exit__(self, type, value, tb):
        self.unswitch()

    def switch(self):
        """ Switch to the requested controllers.

        Any controllers that conflict with the resources used by the requested
        controllers are unloaded. This operation is performed atomically.

        @return tuple containing the loaded and unloaded controllers
        @rtype  [str], [str]
        """
        from controller_manager_msgs.srv import SwitchControllerRequest

        controller_infos_msg = self._mode_switcher._list_controllers_srv()
        controller_infos = controller_infos_msg.controller

        # Stop all running controllers.
        stop_controllers = [
            controller_info.name
            for controller_info in controller_infos
            if controller_info.state == 'running' and any(
                r.resources for r in controller_info.claimed_resources)
	]
        logger.info("Stopping controllers: {}".format(stop_controllers))
        

        ok = self._mode_switcher._switch_controllers_srv(
            stop_controllers=stop_controllers,
            strictness=SwitchControllerRequest.STRICT
        )
        if ok:
            return stop_controllers
        else:
            raise adapy.controller_client.SwitchError('Switching controllers failed.')

    def unswitch(self):
        """ Reverts to the controllers loaded before switch() was called. """
        from controller_manager_msgs.srv import SwitchControllerRequest
        
        if self._stopped_controllers is None:
            raise adapy.controller_client.ROSControlError('Unknown state. Did you call __enter__?')

        ok = self._mode_switcher._switch_controllers_srv(
            start_controllers=self._stopped_controllers,
            strictness=SwitchControllerRequest.STRICT
        )

        self._started_controllers = None

        if not ok:
            raise adapy.controller_client.SwitchError('Reverting controllers failed.')

class EmptyContext:
    def __enter__(self):
        pass

    def __exit__(self, type, value, tb):
        pass

def pause_controls(robot):
    if robot.simulated:
        return EmptyContext()
    else:
        return ControllerPauser(robot._controller_client)


