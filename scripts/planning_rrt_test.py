#/usr/bin/env python
import time
from hpp.corbaserver.motion_prior.client import Client as MPClient
import rospy
from hrp2 import Robot 
from hpp_ros import ScenePublisher, PathPlayer
from hpp.corbaserver import ProblemSolver

print "load mpc model..."
mpc = MPClient()
pc = mpc.precomputation

robot = Robot ()
robot.setTranslationBounds (-6, 6, -6, 6, 0, 1)
client = robot.client
q1 = robot.getInitialConfig()
q2 = robot.getInitialConfig()
q1[0]=0
q1[1]=0
q1[2]=0.7

q2[0]=3
q2[1]=0
q2[2]=0.7

publisher = ScenePublisher(robot)

solver = ProblemSolver (robot)
solver.setInitialConfig (q1)
solver.addGoalConfig (q2)
solver.loadObstacleFromUrdf("hpp_ros","cylinder")
#solver.selectPathPlanner("DiffusingPlanner")

solver.solve ()

print "replay path"
pathplayer = PathPlayer (client, publisher)
pathplayer(1)
