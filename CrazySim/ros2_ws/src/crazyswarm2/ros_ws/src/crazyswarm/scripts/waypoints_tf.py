"""Single CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np

from crazyflie_py import Crazyswarm


Z = 1.0
Z_2 = 2.0
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 3.0
WAYPOINTS = np.array([
    (1.0, 0.0, Z),
    (1.0, 1.0, Z),
    (0.0, 1.0, Z),
    (0.0, 0.0, Z),
])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    # cf = swarm.allcfs.crazyflies[0]
    allcf=swarm.allcfs

    allcf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)
    
    for p in WAYPOINTS:
        i=0
        print(p)
        while i < 4:
            cf=allcf.crazyflies[i]
            cf.goTo(cf.initialPosition + p, yaw=0.0, duration=GOTO_DURATION)
            # timeHelper.sleep(GOTO_DURATION + 1.0)
            i=i+1
        timeHelper.sleep(GOTO_DURATION + 1.0)

    allcf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()
