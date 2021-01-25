#!/usr/bin/env python

import adapy
import argparse
import numpy as np
import openravepy
import prpy, prpy.rave
import rospy
import sensor_msgs.msg

try:
    input = raw_input
except NameError:
    pass

class TrajectoryRecorder:
    def __init__(self, robot, initial_traj=None):
        env = robot.GetEnv()
        self.robot = robot
        self.cspec = robot.GetActiveConfigurationSpecification('linear')
        self.traj = openravepy.RaveCreateTrajectory(env, '')
        self.traj.Init(self.cspec)
        if initial_traj:
            wps = initial_traj.GetWaypoints(0, initial_traj.GetNumWaypoints(), self.cspec)
            self.traj.Insert(0, wps)
            
    def add_waypoint(self, joint_vals):
        waypoint = np.zeros(self.cspec.GetDOF())
        self.cspec.InsertJointValues(waypoint, joint_vals, self.robot,
                                self.robot.GetActiveDOFIndices(), False)
        self.traj.Insert(self.traj.GetNumWaypoints(), waypoint.ravel())
        return waypoint

    def add_current_pos(self):
        return self.add_waypoint(self.robot.GetActiveDOFValues())

    def finish(self):
        return self.traj

def record_trajectory(robot, start_from=None):
    recorder = TrajectoryRecorder(robot, start_from)
    print('Move the robot to the next keypoint and press <enter>. When you are finished, press Ctrl-D to end.')
    try:
        while True:
            input()
            wp = recorder.add_current_pos()
            print('Added waypoint at {}'.format(wp))
    except EOFError:
        traj = recorder.finish()
        print('Finished trajectory with {} waypoints'.format(traj.GetNumWaypoints()))
        return traj
    except Exception as ex:
        import IPython; IPython.embed()
        raise

def get_ee_pose_from_traj(robot, traj):
    with prpy.clone.Clone(robot.GetEnv()) as cloned_env:
        cloned_robot = cloned_env.Cloned(robot)
        cspec = traj.GetConfigurationSpecification()
        dof_indices, _ = cspec.ExtractUsedIndices(robot)
        cloned_robot.SetActiveDOFs(dof_indices)

        for i in range(traj.GetNumWaypoints()):
            waypoint = traj.GetWaypoint(i)
            goal_config = cspec.ExtractJointValues(waypoint, cloned_robot, dof_indices)
            cloned_robot.SetActiveDOFValues(goal_config)
            yield cloned_robot.GetActiveManipulator().GetEndEffectorTransform()

def play_traj(robot, traj):
    if not prpy.util.IsAtTrajectoryStart(robot, traj):
        print('Moving to trajectory start')
        cspec = traj.GetConfigurationSpecification()
        waypoint = traj.GetWaypoint(0)
        dof_indices, _ = cspec.ExtractUsedIndices(robot)
        goal_config = cspec.ExtractJointValues(waypoint, robot, dof_indices)
        robot.SetActiveDOFs(dof_indices)
        robot.PlanToConfiguration(goal_config, execute=True, defer=False)
        rospy.sleep(0.5)
    
    print('Executing trajectory')
    try:
        robot.ExecuteTrajectory(traj, defer=False)
    except ValueError:
        robot.ExecutePath(traj, defer=False)

def main():
    parser = argparse.ArgumentParser(description="Record robot trajectory")
    parser.add_argument('--teleop', action='store_true', help="Enable teleop control for robot (on /ada/joy)")
    parser.add_argument('--save', default='traj.xml', help="output file to save to")
    parser.add_argument('--viewer', default=None, help="viewer to use")
    parser.add_argument('--real', action='store_true', help="real robot instead of sim")
    parser.add_argument('--load', help="input file to use instead of re-recording")
    parser.add_argument('--append', action='store_true', help='append to existing trajectory (only relevant with --load)')
    parser.add_argument('--process', action='store_true', help="postprocess trajectory")
    parser.add_argument('--playback', action='store_true', help="play back trajectory on robot")
    parser.add_argument('--visualize', action='store_true', help="visualize end effector points")
    parser.add_argument('--interactive', action='store_true', help="enter IPython console")
    args = parser.parse_args()

    rospy.init_node("traj_recorder", anonymous=True)

    env, robot = adapy.initialize(attach_viewer=args.viewer, sim=not args.real)
    robot.SetActiveDOFs(robot.arm.GetIndices())

    traj = None
    if args.load:
        traj = prpy.rave.load_trajectory(env, args.load)
        if args.append:
            play_traj(robot, traj)

    teleop_thread = None
    # has to be after the initial playback so we're in an ok controller state
    if args.teleop:
        # this code should really be async-ed but whatever
        import ada_teleoperation.AdaTeleopHandler
        import ada_teleoperation.UserInputMapper
        handler = ada_teleoperation.AdaTeleopHandler.AdaTeleopHandler(
            env, robot, ada_teleoperation.AdaTeleopHandler.kinova_joy_interface_name, 
            ada_teleoperation.UserInputMapper.get_profile("joystick_base_3d"))
        import threading
        def is_done_ros(*args):
            return rospy.is_shutdown()
        # make sure we have a joystick attached
        ok = False
        while not ok:
            try:
                rospy.wait_for_message('/ada/joy', sensor_msgs.msg.Joy, 0.5)
                ok = True
            except rospy.ROSException:
                rospy.logwarn_throttle(5, 'No joystick detected, waiting for initial message on /ada/joy')
        teleop_thread = threading.Thread(target=handler.ExecuteDirectTeleop, args=(is_done_ros,))
        teleop_thread.setDaemon(True)
        # wait a bit for the first command to be received (hack bc this code sucks)
        rospy.sleep(0.2)
        teleop_thread.start()
    
    if not args.load or args.append:
        traj = record_trajectory(robot, traj)

    if args.process:
        traj = robot.PostProcessPath(traj)
    
    if args.save:
        with open(args.save, 'w') as f:
            f.write(traj.serialize())

    if args.visualize:
        ee_poses = np.dstack(list(get_ee_pose_from_traj(robot, traj)))
        ee_positions = ee_poses[:3,3,:].T

        # save the handles so they don't gc
        h_pts = env.plot3(ee_positions, 15.)
        h_lines = env.drawlinestrip(ee_positions, 3.)
    
    if args.playback:
        play_traj(robot, traj)
    
    if args.interactive:
        import IPython
        IPython.embed()
    else:
        rospy.spin()

    if teleop_thread:
        teleop_thread.join()
        
if __name__ == "__main__":
    main()

