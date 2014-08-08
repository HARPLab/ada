'''
Provides a simple console that sets up basic functionality for using adapy and openravepy.
'''
import adapy, logging, numpy, openravepy, sys

if __name__ == "__main__":
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging();

    simulated = 'sim' in sys.argv
    viewer = 'viewer' in sys.argv
    debug = 'debug' in sys.argv

    if debug:
	openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env, robot = adapy.initialize(sim=simulated, attach_viewer=viewer)
    