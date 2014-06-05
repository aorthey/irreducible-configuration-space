#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
for i in range(1,100):
  #project to irreducibility => compute capsules => project capsules onto
  #plane => compute convex hull of capsules => display convex hull
  pv.setRandomConfig()
  pv.projectConfigurationUntilIrreducibleConstraint()
  pv.displayRobot()
  pv.displayConvexHullOfProjectedCapsules()
