#!/usr/bin/env python
import logging, prpy, openravepy, adapy

URDF_PATH = 'package://ada_description/robots/mico.urdf'
SRDF_PATH = 'package://ada_description/robots/mico.srdf'

JACO_URDF_PATH = 'package://ada_description/robots/jaco.urdf'
JACO_SRDF_PATH = 'package://ada_description/robots/jaco.srdf'

def initialize(env_path=None, attach_viewer=False, robot='mico', **kw_args):
    from adarobot import ADARobot
    from util import AdaPyException, find_adapy_resource

    prpy.logger.initialize_logging()

    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
        if not env.Load(env_path):
            raise IOError(
                'Unable to load environment "{:s}".'.format(env_path))

    # Use or_urdf to load ADA from URDF and SRDF.
    with env:
        or_urdf = openravepy.RaveCreateModule(env, 'urdf')
        if robot == 'mico':
            ada_name = or_urdf.SendCommand(
                'load {:s} {:s}'.format(URDF_PATH, SRDF_PATH))
        elif robot == 'jaco':
            ada_name = or_urdf.SendCommand(
                'load {:s} {:s}'.format(JACO_URDF_PATH, JACO_SRDF_PATH))
        else:
            raise RuntimeError('Unknown robot: {} (should be "jaco" or "mico")'.format(robot))

    robot = env.GetRobot(ada_name)
    if robot is None:
        raise AdaPyException('Failed loading ADA with or_urdf.')

    # Bind AdaPy-specific functionality on the robot.
    prpy.bind_subclass(robot, ADARobot, **kw_args)

    # Start by attempting to load or_rviz.
    openravepy.RaveLogInfo('Loading viewer')
    if attach_viewer == True:
        attach_viewer = 'rviz'
        env.SetViewer(attach_viewer)

        # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
           openravepy.RaveLogWarn('Loading rviz failed. Falling back on qt_coin.')
           attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            # spin lock to allow viewer time to build
            built = False
            import time
            for i in range(50):
                time.sleep(0.1)
                if env.GetViewer() is not None:
                    built = True
                    openravepy.RaveLogWarn("RVIZ creation took {} s".format(i*.1))
                    break
            if not built:
                raise AdaPyException(
                    'Failed creating viewer of type "{:s}".'.format(attach_viewer))
    
    # Remove the ROS logging handler again. It might have been added when we
    # loaded or_rviz.
    prpy.logger.remove_ros_logger()

    import adapy.action # register actions
    import adapy.tsr # register TSR libraries
    robot.actions = prpy.action.ActionLibrary()    

    return env, robot

