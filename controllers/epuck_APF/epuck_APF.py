from controller import Robot, Motor, Supervisor, Node
import math
import numpy
import matplotlib.pyplot as plt
import pandas as pd
import os


# time in [ms] of a simulation step
TIME_STEP = 64
v = 0.07 #linear velocity
r = 0.0205 #wheel radius
d = 0.052 #robot diameter
MAX_SPEED = 6.28

# ---- ADDED: patrol mission settings (change these freely) ----
WAYPOINTS = [(1.5, -0.9), (1.5, 0.3), (0.2, 1.6), (-1.3, 1.5), (-1.5, 0.3), (-0.2, -1.5), (0.4, 0.2)]  # (x, z) in metres, visited in this order
WAYPOINT_TOLERANCE = 0.15  # [m] a waypoint counts as reached when the robot is this close (see README_FIXES.txt)
LEG_TIMEOUT = 150.0        # [s] give up on a waypoint and move on if it takes longer than this
SHOW_MARKERS = True        # red balls on the floor at the waypoints; each one disappears when visited
SHOW_TRAIL = True          # blue dots left on the floor behind the robot
TRAIL_EVERY = 0.5          # [s] time between trail dots

BOX_HALF = 0.1      # [m] half the side of a wooden box (boxes are 0.2 m wide)
CYL_RADIUS = 0.1    # [m] radius of a cylinder obstacle
ROBOT_RADIUS = 0.037  # [m] the e-puck is about 7.4 cm wide

def mag(Xi, Zi, Xf, Zf):
    d = math.sqrt((Xi - Xf)**2 + (Zi - Zf)**2)
    return d

robot = Supervisor()

# ADDED: read the obstacles from the world, so the world file is the only place that holds them.
# Every WoodenBox is a box obstacle, every other Solid (except our own "decoration") is a cylinder obstacle.
# To change the layout: move/add/delete obstacles in Webots, save the world, run again.
x = []          # obstacle x coordinates
z = []          # obstacle z coordinates
obs_kind = []   # "box" or "cylinder"
_children = robot.getRoot().getField("children")
for _i in range(_children.getCount()):
    _node = _children.getMFNode(_i)
    if _node.getTypeName() == "WoodenBox":
        _kind = "box"
    elif _node.getTypeName() == "Solid" and _node.getField("name").getSFString() != "decoration":
        _kind = "cylinder"
    else:
        continue
    _p = _node.getField("translation").getSFVec3f()
    x.append(_p[0])
    z.append(_p[2])
    obs_kind.append(_kind)
n_obs = len(x)
print("Obstacles read from the world: %d (%d boxes, %d cylinders)" % (n_obs, obs_kind.count("box"), obs_kind.count("cylinder")))

pos = robot.getFromDef("coordinate")
trans_val = pos.getField("translation")
rot_val = pos.getField("rotation")

x_coord = []
z_coord = []
x_g_rep_pf = []
z_g_rep_pf = []
x_g_at_pf = []
z_g_at_pf = []
x_pf = []
z_pf = []
time = []
heading_angle = []
omega = []
l_speed = []
r_speed = []

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# ======================= ADDED: patrol mission =======================
root_children = robot.getRoot().getField("children")  # list of everything in the world

def add_marker(wx, wz):
    # a red ball with no physics: only decoration, the robot drives through it
    root_children.importMFNodeFromString(-1,
        "Solid { translation %f 0.06 %f name \"decoration\" children [ Shape { appearance PBRAppearance { baseColor 1 0.1 0.1 "
        "emissiveColor 1 0.1 0.1 roughness 1 metalness 0 } geometry Sphere { radius 0.06 } } ] }" % (wx, wz))
    return root_children.getMFNode(root_children.getCount() - 1)  # the node we just added (the last child)

def add_trail_dot(px, pz):
    root_children.importMFNodeFromString(-1,
        "Solid { translation %f 0.015 %f name \"decoration\" children [ Shape { appearance PBRAppearance { baseColor 0.1 0.5 1 "
        "roughness 1 metalness 0 } geometry Sphere { radius 0.015 } } ] }" % (px, pz))

def show_status(text):
    robot.setLabel(0, text, 0.01, 0.93, 0.06, 0xFFFF00)  # text drawn over the 3D view (bottom left, clear of the camera preview)

wp_index = 0           # which waypoint we are driving to now
leg_start_step = 0     # index (in the logged lists) where the current leg started
leg_start_time = 0.0
next_trail_time = 0.0
wp_dist = []           # distance to the CURRENT waypoint at every step
legs = []              # one record per waypoint: number, position, status, first and last logged step
markers = [add_marker(wx, wz) for (wx, wz) in WAYPOINTS] if SHOW_MARKERS else []
# =====================================================================

class epuc:

    def __init__(self, x_f, z_f): #initialize goal coordinates
        self.x_f = x_f
        self.z_f = z_f

    def set_params(self, x_i, z_i, Ka, Kr, xGradRepPotFieldTemp, zGradRepPotFieldTemp):
        self.x_i = x_i
        self.z_i = z_i
        self.Ka = Ka
        self.Kr = Kr
        self.xGradRepPotFieldTemp = xGradRepPotFieldTemp
        self.zGradRepPotFieldTemp = zGradRepPotFieldTemp

    def heading_angle_val(self, field_angle_temp, xGradPotField): #calculates heading angle in robot coordinate space
        if xGradPotField >= 0:
            self.heading_angle = field_angle_temp - 3.14
        else :
            self.heading_angle = field_angle_temp + 3.14

    def speed(self, r=0.0205, d=0.0502, v=0.05): #calculates wheel speed
        self.l = (2*v - d*self.w)/(2*r)
        self.r = (2*v + d*self.w)/(2*r)

    def rep_pf(self, x_pos, z_pos): #Khatib's repulsive potential field
        self.x_pos = x_pos
        self.z_pos = z_pos
        self.DistGoal = mag(self.x_pos,self.z_pos,self.x_f,self.z_f)
        for i in range(n_obs):
            DistObs = mag(self.x_pos,self.z_pos,x[i],z[i])
            do = 1.0
            safe_dist = max(DistObs, 0.03)
            safe_goal_dist = max(self.DistGoal, 0.001)
            if DistObs <= do:
                Gx1 = (self.Kr*((1/safe_dist)-(1/do))*(1/safe_dist**3)*(self.DistGoal)*(self.x_pos - x[i]))
                Gx2 = (0.5*self.Kr*(((1/safe_dist)-(1/do))**2)*(1/safe_goal_dist)*(self.x_pos - self.x_f))
                Gz1 = (self.Kr*((1/safe_dist)-(1/do))*(1/safe_dist**3)*(self.DistGoal)*(self.z_pos - z[i]))
                Gz2 = (0.5*self.Kr*(((1/safe_dist)-(1/do))**2)*(1/safe_goal_dist)*(self.z_pos - self.z_f))
                self.xGradRepPotFieldTemp = (Gx1-Gx2) + self.xGradRepPotFieldTemp
                self.zGradRepPotFieldTemp = (Gz1-Gz2) + self.zGradRepPotFieldTemp

    def att_pf(self):
        self.xGradAttPotField = -self.Ka*2*(self.DistGoal)*(self.x_pos - self.x_f)
        self.zGradAttPotField = -self.Ka*2*(self.DistGoal)*(self.z_pos - self.z_f)
        AttPotField = 1.0*self.DistGoal**2
    
    def angular_vel(self):
        w_temp = (self.heading_angle - a)
        # ADDED: wrap the heading error into [-pi, pi]. Without this, a target direction near +-pi
        # (the robot has to drive east, along +z) makes the error jump by 2*pi and the robot
        # chatters left and right without getting anywhere.
        w_temp = math.atan2(math.sin(w_temp), math.cos(w_temp))
        if w_temp <= 2.0 and w_temp >= -2.0 :
            self.w = 1.5*w_temp
        elif w_temp <= 3.14 and w_temp >= -3.14 :
            self.w = 0.9*w_temp
        else :
            self.w = 0.45*w_temp

    def plot(self):#plotting
        plt.figure() #ADDED: fresh figure, so it does not draw on top of the metrics figure
        # ADDED: one colour per leg, so you can see which part of the path belongs to which waypoint
        colours = plt.cm.viridis(numpy.linspace(0, 1, max(len(legs), 1)))
        for leg, col in zip(legs, colours):
            s, e = leg["i_start"], leg["i_end"] + 1
            plt.plot(z_coord[s:e], x_coord[s:e], color=col, linewidth=2)
        for n, (wx, wz) in enumerate(WAYPOINTS, start=1):
            plt.scatter(wz, wx, marker='*', s=180, color='purple', zorder=5)
            plt.annotate(str(n), (wz, wx), textcoords="offset points", xytext=(7, 7), fontsize=11, color='purple')
        plt.scatter(z_coord[0], x_coord[0], marker='o', color='green')
        plt.scatter(z_coord[-1], x_coord[-1], marker='o', color='red')
        plt.xlabel("Z axis")
        plt.ylabel("X axis")
        plt.title("Patrol path through %d waypoints (green = start, red = end)" % len(WAYPOINTS))
        plt.axis([-2, 2, -2, 2])
        for i in range(n_obs): #ADDED: draw every obstacle at its true size
            if obs_kind[i] == "box":
                plt.gca().add_patch(plt.Rectangle((z[i]-BOX_HALF, x[i]-BOX_HALF), 2*BOX_HALF, 2*BOX_HALF, fc='r'))
            else:
                plt.gca().add_patch(plt.Circle((z[i], x[i]), radius=CYL_RADIUS, fc='y'))
        plt.grid(True)
        plt.savefig(os.path.join(RESULTS_DIR, "path.png"), dpi=120) #ADDED
        plt.show()

    def print_vals(self, field_angle, xGradPotField, zGradPotField):
        print("field angle:", field_angle)
        print("omega:", self.w)
        print("Att Pot Field X:", self.xGradAttPotField)
        print("Att Pot Field Z:", self.zGradAttPotField)
        print("Rep Pot Field X:", self.xGradRepPotFieldTemp)
        print("Rep Pot Field Z:", self.zGradRepPotFieldTemp)
        print("Pot Field X:", xGradPotField)
        print("Pot Field Z:", zGradPotField)

# ======================= ADDED: results and metrics =======================
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(RESULTS_DIR, exist_ok=True)
def surface_distance(px, pz, i):
    # distance from the point (px, pz) to the SURFACE of obstacle i (negative = inside it)
    if obs_kind[i] == "box":
        dx = numpy.abs(px - x[i]) - BOX_HALF
        dz = numpy.abs(pz - z[i]) - BOX_HALF
        return numpy.hypot(numpy.maximum(dx, 0), numpy.maximum(dz, 0)) + numpy.minimum(numpy.maximum(dx, dz), 0)
    return numpy.hypot(px - x[i], pz - z[i]) - CYL_RADIUS

def compute_metrics():
    xs = numpy.array(x_coord)
    zs = numpy.array(z_coord)
    ts = numpy.array(time)
    dist_wp = numpy.array(wp_dist)

    # gap between the robot's body and every obstacle, at every step (negative = they touched)
    gaps = numpy.array([surface_distance(xs, zs, i) - ROBOT_RADIUS for i in range(n_obs)])
    gap_nearest = gaps.min(axis=0)
    step_len = numpy.hypot(numpy.diff(xs), numpy.diff(zs))  # length of every small move
    path_length = step_len.sum()

    lines = ["%-4s %-16s %-10s %9s %9s %11s" % ("Leg", "Waypoint (x, z)", "Status", "Time [s]", "Path [m]", "Efficiency")]
    leg_no = numpy.zeros(len(xs), dtype=int)  # which leg every logged step belongs to
    for leg in legs:
        s, e = leg["i_start"], leg["i_end"]
        leg_no[s:e + 1] = leg["n"]
        leg_time = ts[e] - ts[s]
        leg_path = step_len[s:e].sum()
        straight = mag(xs[s], zs[s], xs[e], zs[e])  # how far the robot really got in this leg
        eff = straight / leg_path if leg_path > 0 else 0.0
        lines.append("%-4d (%5.2f, %5.2f)    %-10s %9.1f %9.2f %11.2f" %
                     (leg["n"], leg["x"], leg["z"], leg["status"], leg_time, leg_path, eff))
    reached = sum(1 for leg in legs if leg["status"] == "reached")
    lines.append("")
    lines.append("%-42s %d of %d" % ("Waypoints reached", reached, len(WAYPOINTS)))
    lines.append("%-42s %.1f" % ("Total time [s]", ts[-1]))
    lines.append("%-42s %.2f" % ("Total path length [m]", path_length))
    lines.append("%-42s %.3f" % ("Average speed [m/s]", path_length / ts[-1]))
    lines.append("%-42s %.3f" % ("Closest gap to any obstacle [m]", gap_nearest.min()))
    lines.append("%-42s %s" % ("Collisions (robot body touched an obstacle)", "none" if gap_nearest.min() > 0 else "YES"))
    near = [i for i in range(n_obs) if gaps[i].min() < 0.30]
    lines.append("%-42s %d of %d" % ("Obstacles passed within 0.30 m", len(near), n_obs))
    lines.append("")
    lines.append("%-4s %-9s %-16s %s" % ("Obs", "Type", "Position (x, z)", "Closest gap [m]"))
    for i in range(n_obs):
        lines.append("%-4d %-9s (%5.2f, %5.2f)    %.3f" % (i + 1, obs_kind[i], x[i], z[i], gaps[i].min()))
    lines.append("%-42s %.2f" % ("Max wheel speed [rad/s]", max(max(l_speed), max(r_speed))))
    with open(os.path.join(RESULTS_DIR, "summary.txt"), "w") as f:
        for line in lines:
            print(line)
            f.write(line + "\n")

    # every logged value, one row per simulation step (open it in Excel)
    pd.DataFrame({"time_s": ts, "x": xs, "z": zs, "leg": leg_no, "dist_waypoint": dist_wp,
                  "gap_to_nearest_obstacle": gap_nearest, "omega": omega,
                  "left_wheel": l_speed, "right_wheel": r_speed}
                 ).to_csv(os.path.join(RESULTS_DIR, "run_log.csv"), index=False)

    # graphs against time
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 9))
    ax[0].plot(ts, dist_wp, label="distance to current waypoint")
    ax[0].plot(ts, gap_nearest, label="gap between robot and nearest obstacle")
    for leg in legs:  # dashed line every time a waypoint is reached or skipped
        ax[0].axvline(ts[leg["i_end"]], color="grey", linestyle="--", linewidth=0.8)
    ax[0].set_ylabel("metres"); ax[0].legend(loc="upper right"); ax[0].grid(True)
    ax[1].plot(ts, omega, color="green")
    ax[1].set_ylabel("omega [rad/s]"); ax[1].grid(True)
    ax[2].plot(ts, l_speed, label="left wheel")
    ax[2].plot(ts, r_speed, label="right wheel")
    ax[2].set_ylabel("wheel speed [rad/s]"); ax[2].set_xlabel("time [s]")
    ax[2].legend(loc="upper right"); ax[2].grid(True)
    fig.suptitle("Patrol run metrics (dashed lines = waypoint reached)")
    fig.savefig(os.path.join(RESULTS_DIR, "metrics.png"), dpi=120)
# ===========================================================================

while robot.step(TIME_STEP) != -1:
	
    bot = epuc(WAYPOINTS[wp_index][0], WAYPOINTS[wp_index][1]) #ADDED: the goal is now the current waypoint
    bot.set_params(-1.8, -0.9, 5.0, 0.5, 0.0, 0.0) #(Xi, Zi, Ka, Kr, xGradpf, zGradpf)

    values = trans_val.getSFVec3f() #retrieves position coordinates
    x_pos = values[0]
    y_pos = values[1]
    z_pos = values[2]
    x_coord.append(x_pos)
    z_coord.append(z_pos)
    t = robot.getTime()
    time.append(t)
    wp_dist.append(mag(x_pos, z_pos, bot.x_f, bot.z_f)) #ADDED
    if SHOW_TRAIL and t >= next_trail_time: #ADDED
        add_trail_dot(x_pos, z_pos)
        next_trail_time = t + TRAIL_EVERY
    # print(z_pos)
    # Heading of the robot about the vertical (y) axis.
    # Webots R2025a's e-puck is built with z as 'up' and is rotated 90 deg in this world,
    # so the heading cannot be read from rot_val[3] any more. It is taken from the
    # orientation matrix instead, offset by pi/2 to match the angle convention used below.
    orient = pos.getOrientation() #3x3 rotation matrix, row by row
    a_raw = math.atan2(-orient[6], orient[0]) - math.pi/2
    a = round(math.atan2(math.sin(a_raw), math.cos(a_raw)), 4) #wrapped to [-pi, pi]
    # print(a)

    bot.rep_pf(x_pos, z_pos)
    bot.att_pf()

    xGradPotField = bot.xGradRepPotFieldTemp + bot.xGradAttPotField
    zGradPotField = bot.zGradRepPotFieldTemp + bot.zGradAttPotField

    field_angle_temp = math.atan2(xGradPotField, zGradPotField)
    # field_angle = robot_coordinate(field_angle_temp,x_pos,z_pos)
    bot.heading_angle_val(field_angle_temp, xGradPotField)
    # field_angle = field_angle_temp + 3.14

    bot.angular_vel()
    heading_error = abs(math.atan2(math.sin(bot.heading_angle - a), math.cos(bot.heading_angle - a)))
    turn_factor = max(0.55, 1.0 - 0.45 * heading_error / math.pi)
    if n_obs:
        nearest_clearance = min(surface_distance(x_pos, z_pos, i) for i in range(n_obs)) - ROBOT_RADIUS
        obstacle_factor = max(0.55, min(1.0, nearest_clearance / 0.30))
    else:
        obstacle_factor = 1.0
    current_speed = v * min(turn_factor, obstacle_factor)
    bot.speed(v=current_speed)

    x_g_rep_pf.append(bot.xGradRepPotFieldTemp)
    z_g_rep_pf.append(bot.zGradRepPotFieldTemp)
    x_g_at_pf.append(bot.xGradAttPotField)
    z_g_at_pf.append(bot.zGradAttPotField)
    x_pf.append(xGradPotField)
    z_pf.append(zGradPotField)
    heading_angle.append(bot.heading_angle)
    omega.append(bot.w)

    leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, bot.l))
    rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, bot.r))

    l_speed.append(leftSpeed)
    r_speed.append(rightSpeed)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # ADDED: waypoint logic
    reached = mag(x_pos, z_pos, bot.x_f, bot.z_f) < WAYPOINT_TOLERANCE
    timed_out = (t - leg_start_time) > LEG_TIMEOUT
    if reached or timed_out:
        status = "reached" if reached else "skipped"
        legs.append({"n": wp_index + 1, "x": bot.x_f, "z": bot.z_f, "status": status,
                     "i_start": leg_start_step, "i_end": len(x_coord) - 1})
        print("Waypoint %d of %d %s at t = %.1f s" % (wp_index + 1, len(WAYPOINTS), status, t))
        if SHOW_MARKERS:
            markers[wp_index].remove()  # the ball disappears
        wp_index += 1
        leg_start_step = len(x_coord) - 1
        leg_start_time = t
        if wp_index == len(WAYPOINTS):
            show_status("Mission complete")
            leftMotor.setVelocity(0.0)   # stop the robot
            rightMotor.setVelocity(0.0)
            robot.step(TIME_STEP)
            break
    progress = 100.0 * wp_index / len(WAYPOINTS)
    show_status("Waypoints %d/%d (%.0f%%)   t = %.0f s" % (wp_index, len(WAYPOINTS), progress, t))
    
compute_metrics() #ADDED
bot.plot()

