E-puck obstacle avoidance with Artificial Potential Fields - patrol mission in Webots R2025a
===========================================================================================
Based on: https://github.com/waynepaulmartis/e-puck-APF-based-static-obstacle-avoidance-using-Webots
(credit the original author in your report)

HOW TO RUN
1. Unzip this folder anywhere (short path, e.g. D:\robotics\apf_project_final).
2. Webots: File > Open World > worlds\epuck static APF.wbt
3. Press Play. Use the Fast button (double triangle) to watch it quicker; the run is ~290 s of simulated time.
4. When the mission is complete two plot windows open. Close them to end the run.
Needs: Webots R2025a, Python 3.12 with numpy, matplotlib, pandas.
In Webots: Tools > Preferences > General > Python command = full path to python.exe.

WHAT YOU SEE
- The robot drives to 7 waypoints in order and avoids 9 obstacles (5 boxes, 4 cylinders) on the way.
- Red balls = waypoints (they disappear when visited), blue dots = trail, yellow text (bottom left) = progress.
- Outputs are saved in controllers\epuck_APF\results:
    summary.txt  per-leg table, totals, closest gap to every obstacle, collision check
    run_log.csv  every logged value at every step (open in Excel)
    metrics.png  gap to nearest obstacle, distance to waypoint, turning rate, wheel speeds
    path.png     the path, one colour per leg, numbered waypoints, obstacles at true size

SETTINGS (top of controllers\epuck_APF\epuck_APF.py)
- WAYPOINTS            the route, (x, z) in metres. Keep every waypoint about 0.5 m or more away from obstacles.
- WAYPOINT_TOLERANCE   0.15 m: a waypoint counts as reached inside this distance.
- LEG_TIMEOUT          150 s: give up on a waypoint and move on.
- SHOW_MARKERS, SHOW_TRAIL, TRAIL_EVERY
- Ka = 5 and Kr = 0.5 (in set_params call), do = 1.0 (influence range, in rep_pf), v = 0.05 m/s.
CHANGING THE OBSTACLES: move, add or delete boxes (WoodenBox) and cylinders (Solid) in Webots, save the world (Ctrl+S).
The controller reads them from the world at start-up. Keep cylinders static (no physics) and boxes 0.2 m wide.

WHAT WAS CHANGED FROM THE ORIGINAL (and why)
World file
- The original world was made in Webots R2021a (y is up). R2025a builds the arena, boxes, cylinders and e-puck with
  z as up, so the floor stood on its edge and the robot fell through it. Added "rotation 1 0 0 -1.5708" to the arena,
  the boxes, the cylinders and the e-puck. Cylinders sit at y = 0.05 and have no physics (static obstacles).
- 4 extra obstacles placed on the route so that avoidance is needed on most legs.
Controller
- Heading is read from getOrientation() instead of rotation[3] (the rotation field is no longer a simple turn about y).
- Heading error is wrapped into [-pi, pi] in angular_vel(). Without it, driving east (+z) makes the error jump by 2*pi
  and the robot chatters left and right forever.
- Plot bug fixed (undefined variable k).
- Added: waypoint patrol with timeout, obstacles read from the world, markers/trail/status label, metrics and result files.
- The potential-field maths (attractive and repulsive forces, wheel-speed equations) is the original author's.

KNOWN LIMITS
- The repulsive term uses D_goal^1, so it does not vanish at a goal that lies inside an obstacle's influence range
  (the GNRON problem). The robot then stops ~0.1 m short; this is why the tolerance is 0.15 m.
- Potential fields can get trapped in local minima (e.g. a U-shaped wall between robot and goal). LEG_TIMEOUT makes the robot move on.
- The robot drives at constant speed and cannot stop, so it can only stop near a waypoint, never exactly on it.

EXTRA IMPROVEMENTS
- The robot slows down when it needs to turn sharply, which can make turns smoother.
- Forward speed also decreases when the robot gets close to an obstacle, giving the controller more time to steer away.
- Minimum distance guards reduce the risk of division-by-zero errors if the robot gets extremely close to an obstacle or goal.
- Left and right wheel speeds are limited by MAX_SPEED so the controller does not request speeds above the configured limit.
- The on-screen status shows the number of completed waypoints and overall mission progress.

These changes keep the original APF navigation approach and are intended to be easy to explain and test in a student project. Run the simulation in Webots to compare the path and summary output before and after the changes.


Speed adjustment
The controller now uses a nominal linear speed of 0.07 m/s instead of 0.05 m/s. It reduces speed less aggressively during turns and near obstacles. Since this is a faster setting, run the simulation and check the collision/clearance report before using it for final evaluation. If the robot clips obstacles, lower v to 0.06 m/s or increase the obstacle slowdown.
