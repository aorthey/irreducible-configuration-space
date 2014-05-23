#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
pv.setRandomConfig()
#pv.setConfig(pv.q0)
#pv.compute()
#pv.displayRobot()
#pv.computeConvexHull()

for i in range(1,10):
        #project to irreducibility => compute capsules => project capsules onto
        #plane => compute convex hull of capsules => display convex hull
        pv.projectConfigurationUntilIrreducible()
        #pv.projectOntoConstraintManifold()
        pv.computeCapsulesFromConfiguration()
        pv.computeConvexHullOfProjectedCapsules()
        pv.displayRobot()
        pv.displayConvexHullOfProjectedCapsules()
