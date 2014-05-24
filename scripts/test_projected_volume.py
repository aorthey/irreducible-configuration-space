#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
pv.setRandomConfig()
#pv.compute()
#pv.displayRobot()
#pv.computeConvexHull()

print pv.precomputation.getNumberDof()
print pv.precomputation.getNumberDof()
print pv.precomputation.getNumberDof()
for i in range(1,10):
        #project to irreducibility => compute capsules => project capsules onto
        #plane => compute convex hull of capsules => display convex hull
        pv.projectConfigurationUntilIrreducible()
        pv.computeCapsulesFromConfiguration()
        pv.computeConvexHullOfProjectedCapsules()
        pv.displayRobot()
        pv.displayConvexHullOfProjectedCapsules()
