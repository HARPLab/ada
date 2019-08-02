#!/usr/bin/env python
import adapy, openravepy, numpy, prpy, rospy
from IPython import embed
import argparse

def PlanToTransform(env, robot, transform):
    handle = openravepy.misc.DrawAxes(env, transform);
    iksolver = robot.arm.GetIkSolver()
    param = openravepy.IkParameterization(transform, openravepy.IkParameterizationType.Transform6D)
    solution = iksolver.Solve(param, robot.GetActiveDOFValues(),  0)
    traj =  robot.PlanToConfiguration(solution.GetSolution())
    return traj;

def PlanToOffset(env, robot, offset):
    transform = robot.arm.GetEndEffectorTransform()
    transform[0:3, 3] += offset;
    traj = PlanToTransform(env, robot, transform);
    return traj

rospy.init_node('test_scenario', anonymous = True)

parser = argparse.ArgumentParser(description='adapy test file')
parser.add_argument('--sim', default=False, action='store_true', help='run in simulation mode')
parser.add_argument('--no-viewer', default=False, action='store_true', help='run without rviz viewer (useful for remote running)')
args = parser.parse_args()

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.misc.InitOpenRAVELogging();

env, robot = adapy.initialize(attach_viewer=('rviz' if not args.no_viewer else None), sim=args.sim)
manip = robot.arm

manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
robot.SetActiveDOFs([2,3,4,5,6,7])
robot.arm.SetActive()
values = robot.GetActiveDOFValues()
values[1] = values[1] - 0.3
robot.PlanToConfiguration(values, execute=True)


from IPython import embed
embed()
