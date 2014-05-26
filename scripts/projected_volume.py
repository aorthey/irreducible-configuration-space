#/usr/bin/env python
import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
#from hpp.corbaserver.client import Client as WsClient
from hpp.corbaserver.wholebody_step.client import Client as WsClient
from hrp2 import Robot
import rospy
import numpy
import math

class ProjectedVolume ():
        def __init__ (self):
                self.robot_interface = Robot ()
                self.robot_interface.setTranslationBounds (-3, 3, -3, 3, 0, 1)
                self.cl = self.robot_interface.client
                self.wcl = WsClient ()

                self.robot = self.cl.robot
                self.precomputation = self.cl.precomputation

                self.scene_publisher = ScenePublisher (self.robot_interface.jointNames [4:])
                self.q0 = self.robot_interface.getInitialConfig ()
                self.q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
                self.q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
                self.q = self.q0
                self.distanceToProjection = 1
                self.vol_cvx_hull = []

        def projectOnConstraintsManifold(self, q_in):

                self.wcl.problem.addStaticStabilityConstraints ("balance", q_in , "LLEG_JOINT5", "RLEG_JOINT5")

                #self.cl.problem.setNumericalConstraints ("balance",
                                #["balance/relative-com",
                                #"balance/relative-orientation",
                                #"balance/relative-position",
                                #"balance/orientation-left-foot",
                                #"balance/position-left-foot"])
                p1 = [0.0,0.0,1.0]
                p2 = [0.0,0.0,1.0]
                self.cl.problem.createPositionConstraint ("foot-on-floor", "LLEG_JOINT5", "RLEG_JOINT5", p1, p2)
                #self.cl.problem.createPositionConstraint ("RARM", "RARM_JOINT5", "", p1, p2)
                #self.cl.problem.createPositionConstraint ("LARM", "LARM_JOINT5", "", p1, p2)

                status, qproj, residual = self.cl.problem.applyConstraints (q_in)
                print "Projection Error on constraints manifold: ",residual," successful: ", status
                return qproj

        def setConfig(self, q_in):
                self.q = q_in
                self.robot.setCurrentConfig(self.q)

        def setRandomConfig(self):
                #set random configuration, but do not change the CoM
                self.q_old = self.q
                self.q = self.robot.getRandomConfig()
                self.q[:8]=self.q_old[:8] 

                self.q = self.projectOnConstraintsManifold(self.q)
                self.setConfig(self.q)

        #def volume_sorted_cvx_hull(self, hull):
        #        #assume that hull are points on a convex hull, which are sorted
        #        # counter-clockwise.
        #        volume = 0
        #        x1 = numpy.array(hull[0][0],hull[0][1])
        #        for i in range(0,len(hull)-1):
        #                x2 = numpy.array(hull[i][0],hull[i][1])
        #                x3 = numpy.array(hull[i+1][0],hull[i+1][1])
        #                #vectors between points
        #                v12 = x2-x1
        #                v13 = x3-x1
        #                #compute base length
        #                b = numpy.linalg.norm(v12)
        #                #compute height by computing distance of x3 to line
        #                #between x1 and x2
        #                h_vec = v13 - numpy.dot(v13,v12)*v12
        #                h = numpy.linalg.norm(h_vec)
        #                d = 0.5*b*h
        #                volume = volume + d
        #        return volume
                

        def displayConvexHullOfProjectedCapsules(self):
                self.hull = self.precomputation.getConvexHullCapsules()
                self.hull = zip(*[iter(self.hull)]*3)
                print self.precomputation.getVolume()

                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher.oid = 0
                #self.scene_publisher.addPolygonFilled(self.hull)
                self.scene_publisher.addPolygon(self.hull, 0.05)
                self.scene_publisher.publishObjects()
                r.sleep()

        def projectConfigurationUntilIrreducible(self):
                self.precomputation.setCurrentConfiguration(self.q)

                self.q_grad = self.precomputation.getGradient()
                self.q_grad_raw = self.q_grad
                self.q_grad.insert(3,0)
                self.q_grad[0]=0
                self.q_grad[1]=0
                self.q_grad[2]=0
                #make sure that quaternions are not changed
                self.q_grad[3]=0
                self.q_grad[4]=0
                self.q_grad[5]=0
                self.q_grad[6]=0
                self.q_new = [x+y for x,y in zip(self.q,self.q_grad)]
                print self.q_new[0:8]
                self.q_new = self.projectOnConstraintsManifold(self.q_new)
                self.setConfig(self.q_new)
                
        def displayRobot(self):
                r = rospy.Rate(1)
                r.sleep()
                self.scene_publisher(self.q)
