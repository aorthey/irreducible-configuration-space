#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
#pv.setConfig(pv.q0)
pv.setRandomConfig()
#pv.compute()
#pv.displayRobot()
#pv.computeConvexHull()

for i in range(1,10):
        pv.computeConvexHullProjection()
        pv.computeCapsuleProjection()
        pv.displayConvexHullProjection()
