#/usr/bin/env python
import time
from hpp.corbaserver.motion_prior.client import Client as MPClient
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import rospy
from hrp2 import Robot 
from hpp_ros import ScenePublisher, PathPlayer

robot = Robot ()
robot.setTranslationBounds (-0.5, 0.5, -3, 3, 0, 1)
client = robot.client

publisher = ScenePublisher(robot)
pathplayer = PathPlayer (client, publisher)

fn="trajectory-wall-direct-path.tau"
pathplayer.fromFile(fn)
