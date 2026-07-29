from pybricks.hubs import MoveHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

hub = MoveHub()

# =====================================================================
# FIXED-POINT INTEGER TRIGONOMETRY (No math module required)
# =====================================================================
# Pre-calculated Sine values scaled by 1000 for 36 steps (10° increments)
# sin(0°..360°) * 1000
SIN_TABLE = [
    0, 174, 342, 500, 643, 766, 866, 940, 985, 1000,
    985, 940, 866, 766, 643, 500, 342, 174, 0,
    -174, -342, -500, -643, -766, -866, -940, -985, -1000,
    -985, -940, -866, -766, -643, -500, -342, -174, 0
]

def get_cos(step):
    # cos(x) = sin(x + 90°) -> shift index by 9 steps (90 deg)
    return SIN_TABLE[(step + 9) % 36]

def get_sin(step):
    return SIN_TABLE[step % 36]

# =====================================================================
# PLOTTER PARAMETERS & CALIBRATION
# =====================================================================
PEN_UP_ANGLE = 0        # Fully raised against endstop
PEN_DOWN_ANGLE = 95     # Angle to lower pen down to paper surface

MOVE_SPEED = 300        # Normal movement speed for X/Y
HOME_SPEED_XY = 150     # Search speed for endstops
HOME_SPEED_Z = 200      # Search speed for pen lift endstop

HOME_DUTY_Z = 30        # Stalling force limit for Z
HOME_DUTY_X = 18        # Stalling force limit for X
HOME_DUTY_Y = 30        # Stalling force limit for Y

BACKLASH_X = 9
BACKLASH_Y = 5

SX_NUM, SX_DEN = 249, 180  # X unit scaling (baseline)
SY_NUM, SY_DEN = 253, 180  # Updated Y unit scaling

X_MIN_BOUND = 0
X_MAX_BOUND = 0
Y_MIN_BOUND = 0
Y_MAX_BOUND = 0

# =====================================================================
# HARDWARE SETUP
# =====================================================================
mx = Motor(Port.D)  # X axis motor
my = Motor(Port.C)  # Y axis motor
mz = Motor(Port.B)  # Pen lift motor

ox, oy = 0, 0
ox_dir, oy_dir = 0, 0

# =====================================================================
# HELPER FUNCTIONS
# =====================================================================
def head_up():
    mz.run_target(abs(HOME_SPEED_Z), PEN_UP_ANGLE, then=Stop.BRAKE, wait=True)

def head_down():
    wait(100)
    mz.run_target(abs(HOME_SPEED_Z), PEN_DOWN_ANGLE, then=Stop.BRAKE, wait=True)
    wait(100)

def move(x, y):
    global ox, oy, ox_dir, oy_dir
    
    rx = x - ox
    ry = y - oy
    
    nx_dir = 1 if rx > 0 else (-1 if rx < 0 else ox_dir)
    ny_dir = 1 if ry > 0 else (-1 if ry < 0 else oy_dir)

    if nx_dir != ox_dir and nx_dir != 0:
        mx.run_angle(abs(MOVE_SPEED), -BACKLASH_X * nx_dir, then=Stop.BRAKE, wait=True)

    if ny_dir != oy_dir and ny_dir != 0:
        my.run_angle(abs(MOVE_SPEED), -BACKLASH_Y * ny_dir, then=Stop.BRAKE, wait=True)

    xb = BACKLASH_X if nx_dir == 1 else 0
    yb = BACKLASH_Y if ny_dir == 1 else 0

    # Target integer degrees (+Y moves UP)
    tx_deg = -(((x + xb) * SX_NUM) // SX_DEN)
    ty_deg = (((y + yb) * SY_NUM) // SY_DEN)

    if rx != 0 and ry != 0:
        mx.run_target(abs(MOVE_SPEED), tx_deg, then=Stop.BRAKE, wait=False)
        my.run_target(abs(MOVE_SPEED), ty_deg, then=Stop.BRAKE, wait=True)
    elif rx != 0:
        mx.run_target(abs(MOVE_SPEED), tx_deg, then=Stop.BRAKE, wait=True)
    elif ry != 0:
        my.run_target(abs(MOVE_SPEED), ty_deg, then=Stop.BRAKE, wait=True)

    ox = x
    oy = y
    ox_dir = nx_dir
    oy_dir = ny_dir

def home():
    global X_MIN_BOUND, X_MAX_BOUND, Y_MIN_BOUND, Y_MAX_BOUND
    print("Homing sequence started...")

    # 1. Home Z
    print("homing Z")
    mz.run_until_stalled(-abs(HOME_SPEED_Z), then=Stop.COAST, duty_limit=HOME_DUTY_Z)
    mz.reset_angle(PEN_UP_ANGLE)
    wait(200)

    # 2. Home X
    print("homing X - finding first endstop...")
    mx.run_until_stalled(abs(HOME_SPEED_XY), then=Stop.COAST, duty_limit=HOME_DUTY_X)
    mx.reset_angle(0)
    wait(200)

    print("homing X - finding opposite endstop...")
    mx.run_until_stalled(-abs(HOME_SPEED_XY), then=Stop.COAST, duty_limit=HOME_DUTY_X)
    x_span = mx.angle()
    wait(200)

    print("centering X axis...")
    mx.run_target(abs(HOME_SPEED_XY), x_span // 2, then=Stop.BRAKE, wait=True)
    mx.reset_angle(0)
    wait(200)

    x_half_units = ((abs(x_span) // 2) * SX_DEN) // SX_NUM
    X_MIN_BOUND = -x_half_units
    X_MAX_BOUND = x_half_units

    # 3. Home Y
    print("homing Y - finding first endstop...")
    my.run_until_stalled(abs(HOME_SPEED_XY), then=Stop.COAST, duty_limit=HOME_DUTY_Y)
    my.reset_angle(0)
    wait(200)

    print("homing Y - finding opposite endstop...")
    my.run_until_stalled(-abs(HOME_SPEED_XY), then=Stop.COAST, duty_limit=HOME_DUTY_Y)
    y_span = my.angle()
    wait(200)

    print("centering Y axis...")
    my.run_target(abs(HOME_SPEED_XY), y_span // 2, then=Stop.BRAKE, wait=True)
    my.reset_angle(0)
    wait(200)

    y_half_units = ((abs(y_span) // 2) * SY_DEN) // SY_NUM
    Y_MIN_BOUND = -y_half_units
    Y_MAX_BOUND = y_half_units

    print("Homing complete!")
    print("X bounds:", X_MIN_BOUND, "to", X_MAX_BOUND)
    print("Y bounds:", Y_MIN_BOUND, "to", Y_MAX_BOUND)

# =====================================================================
# PURE INTEGER CIRCLE DRAWING
# =====================================================================
def draw_circle(radius):
    print("Drawing circle radius:", radius, "units")

    # Step 0: Start at rightmost point
    start_x = (radius * get_cos(0)) // 1000
    start_y = (radius * get_sin(0)) // 1000

    head_up()
    move(start_x, start_y)
    head_down()

    # Steps 1 to 36: Interpolate around circumference
    for step in range(1, 37):
        x = (radius * get_cos(step)) // 1000
        y = (radius * get_sin(step)) // 1000
        move(x, y)

    head_up()

def draw():
    print("--- Starting Concentric Circles Test ---")

    max_half_width = min(abs(X_MIN_BOUND), abs(X_MAX_BOUND))
    max_half_height = min(abs(Y_MIN_BOUND), abs(Y_MAX_BOUND))
    max_radius = min(max_half_width, max_half_height)

    # 1. Target Crosshairs
    head_up()
    move(-max_radius, 0)
    head_down()
    move(max_radius, 0)
    head_up()

    move(0, max_radius)
    head_down()
    move(0, -max_radius)
    head_up()

    # 2. Concentric Circles (25%, 50%, and 90% radius)
    draw_circle((max_radius * 25) // 100)
    draw_circle((max_radius * 50) // 100)
    draw_circle((max_radius * 90) // 100)

    print("--- Concentric Circles Test Complete ---")

# =====================================================================
# EXECUTION
# =====================================================================
home()

draw()

print("Plot complete. Returning to center (0, 0)...")
head_up()
move(0, 0)
