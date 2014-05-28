#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
pv.setRandomConfig()
pv.displayRobot()

for i in range(1,10):
  #project to irreducibility => compute capsules => project capsules onto
  #plane => compute convex hull of capsules => display convex hull
  pv.setRandomConfig()
  pv.projectConfigurationUntilIrreducible()
  pv.displayRobot()
  pv.displayConvexHullOfProjectedCapsules()
