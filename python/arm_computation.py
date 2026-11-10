import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
import time
import colorsys
from matplotlib.patches import Rectangle


                            ###################
           #####################################################
       #######                4 DOF Robot Arm               #######
   #######                Controller & Visualiser               #######
######                                                              ######
   #######                           by                         #######
       #######                Kieran Pettigrew             #######
           ####################################################
                            ###################


# ------------------------------------------
# SETUP
# ------------------------------------------

arduino_port = "/dev/cu.usbmodem2101"
baud_rate = 115200
send_to_arduino = True

interp_steps = 200
ani_interval = 40
hold_steps = 125


# Link lengths
base_height = 40    # vertical height of the base
L1 = 100           # length from shoulder to elbow
L2 = 60            # length from elbow to wrist
L3 = 105            # length from wrist to gripper end

START_ANGLES = [90.0, 110.0, 15.0, 15.0]

TRANSIT = (0, 100, 85, -50)

PICKUP_POSITIONS = {
    "POS_1": (0, 100, 10, -90),
    "POS_2": (-40, 100, 10, -90),
    "POS_3": (40, 100, 10, -90),
}

LEFT_DROP  = (-110, 0, 130, -20)
RIGHT_DROP = (110, 0, 130, -20)

GRIPPER_CLOSED = 54
GRIPPER_START  = 70
GRIPPER_OPEN   = 73

joint_limits = {
    "theta1": (0, 180),     # Base rotation
    "theta2": (30, 150),    # Shoulder
    "theta3": (0, 90),      # Elbow
    "theta4": (0, 90)       # Wrist
}

VIAL_ROIS = {
    "POS_1": (80, 75, 94, 96),
    "POS_2": (120, 75, 134, 96),
    "POS_3": (39, 75, 53, 96),
}

REQUIRED_STABLE_FRAMES = 3

SORT_ACTIONS = {
    "BLUE": "LEFT",
    "RED": "RIGHT",
    "GREEN": "LEAVE"
}








# ------------------------------------------
# KINEMATICS
# ------------------------------------------

def inverse_kinematics(x, y, z, phi):
    # Convert phi to radians
    phi = np.radians(phi)

    # Calculate base rotation angle
    theta1 = np.arctan2(y, x)

    # Calculate the horizontal distance from the base to the target point
    r4 = np.sqrt(x**2 + y**2)
    # Calculate the vertical distance fromt eh base to the target point
    h4 = z - base_height

    # Calculate the position of the wrist (joint 3)
    r3 = r4 - L3 * np.cos(phi)
    # Calculate the vertical position of the wrist (joint 3)
    h3 = h4 - L3 * np.sin(phi)

    # Reachability check
    d = np.sqrt(r3**2 + h3**2)
    if d > (L1 + L2) or d < abs(L1 - L2):
        raise ValueError("Target position is out of reach")
    
    # Calculate elbow angle (theta3) unsing cosine rule
    theta3 = np.pi + np.arccos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))

    # Calculate shoulder angle (theta2)
    theta2 = np.arccos((L1**2 + d**2 - L2**2) / (2 * L1 * d)) + np.arctan2(h3, r3)

    # Calculate wrist angle (theta4)
    theta4 = phi - theta2 - theta3

    # Convert to degrees
    theta1 = np.degrees(theta1)
    theta2 = np.degrees(theta2)
    theta3 = np.degrees(theta3)
    theta4 = np.degrees(theta4)

    # Normalise angles to be within 0-360 degrees
    theta1 = normalise_angle(theta1)
    theta2 = normalise_angle(theta2)
    theta3 = normalise_angle(theta3)
    theta4 = normalise_angle(theta4)
    
    return theta1, theta2, theta3, theta4


def forward_kinematics(theta1, theta2, theta3, theta4):
    # Convert angles from degrees to radians
    t1 = np.radians(theta1)
    t2 = np.radians(theta2)
    t3 = np.radians(theta3)
    t4 = np.radians(theta4)

    # Set base coordinates (origin)
    x0 = 0
    y0 = 0
    z0 = 0
    coord0 = np.array([x0, y0, z0])

    # Calculate the position of the shoulder (joint 1)
    x1 = 0
    y1 = 0
    z1 = base_height
    coord1 = np.array([x1, y1, z1])

    # Calculate the position of the elbow (joint 2)
    x2 = x1 + L1 * np.cos(t1) * np.cos(t2)
    y2 = y1 + L1 * np.sin(t1) * np.cos(t2)
    z2 = z1 + L1 * np.sin(t2)
    coord2 = np.array([x2, y2, z2])

    # Calculate the position of the wrist (joint 3)
    x3 = x2 + L2 * np.cos(t1) * np.cos(t2 + t3)
    y3 = y2 + L2 * np.sin(t1) * np.cos(t2 + t3)
    z3 = z2 + L2 * np.sin(t2 + t3)
    coord3 = np.array([x3, y3, z3])

    # Calculate the position of the gripper end (joint 4)
    x4 = x3 + L3 * np.cos(t1) * np.cos(t2 + t3 + t4)
    y4 = y3 + L3 * np.sin(t1) * np.cos(t2 + t3 + t4)
    z4 = z3 + L3 * np.sin(t2 + t3 + t4)
    coord4 = np.array([x4, y4, z4])

    return np.array([coord0, coord1, coord2, coord3, coord4])








# ------------------------------------------
# VALUE FIXING FOR ARDUINO
# ------------------------------------------

def normalise_angle(angle):
    return angle % 360


def angle_within_limits(angle, lower_limit, upper_limit):
    angle = normalise_angle(angle)

    # Full rotation case
    if lower_limit == 0 and upper_limit == 360:
        return True
    
    lower_limit = normalise_angle(lower_limit)
    upper_limit = normalise_angle(upper_limit)

    # Normal case (30 to 150)
    if lower_limit <= upper_limit:
        return lower_limit <= angle <= upper_limit

    # Wrap-around case (300 to 60)
    return angle >= lower_limit or angle <= upper_limit


def check_joint_limits(theta1, theta2, theta3, theta4):
    angles = {
        "theta1": theta1,
        "theta2": theta2,
        "theta3": theta3,
        "theta4": theta4
    }

    for joint_name, angle in angles.items():
        lower_limit, upper_limit = joint_limits[joint_name]

        if not angle_within_limits(angle, lower_limit, upper_limit):
            raise ValueError(
                f"{joint_name} angle {angle:.2f}° is outside its limit "
                f"({lower_limit}° to {upper_limit}°)"
            )


def relative_angle_to_servo(angle):
    # Convert normalised 0-360° model angle
    # into signed -180° to +180° rotation
    if angle > 180:
        angle = angle - 360

    # Servo neutral is 90°
    servo_angle = 90 + angle

    return servo_angle


def sim_to_real_angle(theta1, theta2, theta3, theta4):
    servo_angles = [
        theta1,
        theta2,
        relative_angle_to_servo(theta3),
        relative_angle_to_servo(theta4)
    ]

    return servo_angles








# ------------------------------------------
# BUILD AND SMOOTH MOTION
# ------------------------------------------

def interpolate_positions(start, end, steps):
    start = np.array(start)
    end = np.array(end)

    positions = []

    for i in range(steps):
        fraction = i / steps
        position = start + fraction * (end - start)
        positions.append(position)

    return positions


# Build full motion path
def build_motion(waypoints):
    motion = []

    # First waypoint
    first_position, first_gripper, first_hold = waypoints[0]

    motion.append((np.array(first_position), first_gripper))

    # Remaining waypoints
    for i in range(len(waypoints) - 1):
        start_position, start_gripper, _ = waypoints[i]
        end_position, end_gripper, end_hold = waypoints[i + 1]

        # Move arm only if Cartesian position changes
        if start_position != end_position:
            positions = interpolate_positions(
                start_position,
                end_position,
                interp_steps
            )

            # Keep current gripper state while travelling
            for position in positions:
                motion.append((position, start_gripper))

        # Hold at destination and apply new gripper state
        hold_frames = int(end_hold * 1000 / ani_interval)

        for _ in range(hold_frames):
            motion.append((np.array(end_position), end_gripper))

    return motion








# ------------------------------------------
# ARDUINO CONNECTION
# ------------------------------------------

def wait_for_camera_startup(arduino):

    print("Waiting for camera startup to finish...")

    STARTUP_MIN_TIME = 6.0
    QUIET_TIME = 1.5
    MAX_WAIT_TIME = 15.0

    start_time = time.monotonic()
    last_data_time = start_time

    discarded_bytes = 0

    while True:
        waiting = arduino.in_waiting

        if waiting > 0:
            data = arduino.read(waiting)
            discarded_bytes += len(data)
            last_data_time = time.monotonic()

        elapsed = time.monotonic() - start_time
        quiet_for = time.monotonic() - last_data_time

        # Do NOT allow startup to finish early.
        # The camera must have had at least 6 seconds,
        # AND the serial line must then be quiet.
        if (
            elapsed >= STARTUP_MIN_TIME
            and quiet_for >= QUIET_TIME
        ):

            arduino.reset_input_buffer()

            print(
                f"Camera startup complete "
                f"({discarded_bytes} startup bytes discarded)"
            )

            return

        if elapsed >= MAX_WAIT_TIME:
            arduino.reset_input_buffer()
            print(
                f"Camera startup wait timed out "
                f"({discarded_bytes} bytes discarded). "
                f"Continuing."
            )

            return

        time.sleep(0.05)


# Arduino serial connection
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)

print("Waiting for Arduino and camera to initialise...")
wait_for_camera_startup(arduino)

arduino.reset_input_buffer()

print("Arduino connected")


last_command = None
def send_servo_command(servo_angles, gripper_angle):
    global last_command

    if not send_to_arduino or arduino is None:
        return

    command = (
        f"{servo_angles[0]:.1f},"
        f"{servo_angles[1]:.1f},"
        f"{servo_angles[2]:.1f},"
        f"{servo_angles[3]:.1f},"
        f"{gripper_angle}\n"
    )

    # Don't repeatedly send exactly the same command
    if command == last_command:
        return

    arduino.write(command.encode("utf-8"))
    arduino.flush()
    last_command = command

    print("Sent:", command.strip())


def return_to_start():
    print("Returning arm and gripper to start position...")

    send_servo_command(
        START_ANGLES,
        GRIPPER_START
    )









# ------------------------------------------
# CAMERA VISION SYSTEM
# ------------------------------------------

def decode_rgb565(pixel_bytes, width, height):
    data = np.frombuffer(pixel_bytes, dtype=np.uint8)
    expected = width * height * 2

    if len(data) != expected:
        raise ValueError(
            f"Expected {expected} pixel bytes, got {len(data)}"
        )

    high = data[0::2].astype(np.uint16)
    low = data[1::2].astype(np.uint16)

    raw = (high << 8) | low

    r = ((raw >> 8) & 0xF8).astype(np.uint8)
    g = ((raw >> 3) & 0xFC).astype(np.uint8)
    b = ((raw << 3) & 0xF8).astype(np.uint8)

    image = np.stack((r, g, b), axis=-1)
    image = image.reshape((height, width, 3))
    # Fixes camera mounted upside down
    image = np.rot90(image, 2)

    return image


def classify_rgb(rgb):
    r, g, b = rgb / 255.0
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    hue = h * 360

    # Reject poorly saturated colours such as white/grey
    if s < 0.35:
        return "UNKNOWN"

    if hue < 30 or hue > 330:
        return "RED"

    if 90 <= hue <= 160:
        return "GREEN"

    if 175 <= hue <= 240:
        return "BLUE"

    return "UNKNOWN"


def analyse_vial_positions(image):
    results = {}

    for name, (x1, y1, x2, y2) in VIAL_ROIS.items():
        roi = image[y1:y2 + 1, x1:x2 + 1]
        pixels = roi.reshape(-1, 3)
        rgb = np.median(pixels, axis=0)
        colour = classify_rgb(rgb)

        results[name] = {
            "rgb": rgb,
            "colour": colour
        }

        print(
            f"{name}: "
            f"{colour:7s} | "
            f"R={rgb[0]:.0f}, "
            f"G={rgb[1]:.0f}, "
            f"B={rgb[2]:.0f}"
        )

    print()

    return results


def create_sorting_plan(detection):
    positions = ["POS_1", "POS_2", "POS_3"]
    sorting_plan = []

    for position, colour in zip(positions, detection):
        action = SORT_ACTIONS[colour]
        sorting_plan.append({
            "position": position,
            "colour": colour,
            "action": action
        })

    return sorting_plan


def build_sorting_waypoints(sorting_plan):

    waypoints = [
        # Start by moving into the safe transit position
        (TRANSIT, GRIPPER_OPEN, 5)
    ]

    for item in sorting_plan:

        position = item["position"]
        colour = item["colour"]
        action = item["action"]

        # Green vial stays where it is
        if action == "LEAVE":
            print(f"{position}: {colour} left in place")
            continue

        pickup = PICKUP_POSITIONS[position]

        pickup_x, pickup_y, pickup_z, pickup_phi = pickup
        pickup_above = (
            pickup_x,
            pickup_y,
            pickup_z + 25,
            -75)

        if action == "LEFT":
            drop = LEFT_DROP

        elif action == "RIGHT":
            drop = RIGHT_DROP

        else:
            raise ValueError(
                f"Unknown sorting action: {action}"
            )

        print(
            f"{position}: picking up {colour} "
            f"and moving it {action}"
        )

        waypoints.extend([
            # Set gripper above vial
            (pickup_above, GRIPPER_OPEN, 1),

            # Approach pickup with gripper open
            (pickup, GRIPPER_OPEN, 1),

            # Stay at pickup and close gripper
            (pickup, GRIPPER_CLOSED, 3),

            # Retract to safe transit position
            (pickup_above, GRIPPER_CLOSED, 1),

            # Move to appropriate drop position
            (drop, GRIPPER_CLOSED, 1),

            # Stay at drop position and release vial
            (drop, GRIPPER_OPEN, 1),

            # Return to safe transit position
            (TRANSIT, GRIPPER_OPEN, 1),
        ])

    return waypoints


COMMAND_NEW_FRAME = 0x01
PIXEL_FORMAT_RGB565 = 1


def capture_camera_frame(arduino):
    # Remove any old READY / OK messages
    arduino.reset_input_buffer()

    # Ask Arduino for exactly one frame
    arduino.write(b"CAM\n")
    arduino.flush()

    command_length = None
    command_data = bytearray()

    width = None
    height = None

    frame_data = bytearray()
    expected_frame_bytes = None
    start_time = time.monotonic()

    while True:
        if time.monotonic() - start_time > 15:
            raise TimeoutError(
                "No complete camera frame received "
                "within 15 seconds."
            )

        incoming = arduino.read(1)

        if not incoming:
            continue

        value = incoming[0]

        # Currently reading camera command/header
        if command_length is not None:
            command_data.append(value)

            if len(command_data) >= command_length + 1:
                data = command_data[:-1]

                if len(data) > 0:
                    command = data[0] & 0x0F

                    if command == COMMAND_NEW_FRAME:
                        if len(data) >= 4:
                            width = (data[1] | ((data[3] & 0x03) << 8))
                            height = (data[2] | ((data[3] & 0x0C) << 6))
                            pixel_format = ((data[3] & 0xF0) >> 4)

                            if pixel_format == PIXEL_FORMAT_RGB565:
                                expected_frame_bytes = (width * height * 2)
                                frame_data = bytearray()

                command_length = None
                command_data = bytearray()

            continue

        # Zero marks start of camera command
        if value == 0:
            length = arduino.read(1)

            if length:
                command_length = length[0]
                command_data = bytearray()

            continue

        # Pixel data
        if expected_frame_bytes is not None:
            frame_data.append(value)

            if len(frame_data) >= expected_frame_bytes:
                return decode_rgb565(
                    frame_data,
                    width,
                    height
                )


def identify_vials(arduino):
    print("\nIdentifying vial colours...")

    last_detection = None
    stable_count = 0

    # CAMERA DISPLAY WINDOW
    camera_fig, camera_ax = plt.subplots()
    camera_ax.set_title("OV7670 - Identifying Vials")
    camera_ax.axis("off")
    camera_display = None

    # Draw vial position boxes
    for name, (x1, y1, x2, y2) in VIAL_ROIS.items():
        rect = Rectangle(
            (x1, y1),
            x2 - x1,
            y2 - y1,
            fill=False,
            linewidth=1.5
        )

        camera_ax.add_patch(rect)
        camera_ax.text(
            x1,
            y1 - 2,
            name,
            fontsize=8
        )

    plt.show(block=False)

    while True:
        image = capture_camera_frame(arduino)

        # UPDATE CAMERA DISPLAY
        if camera_display is None:

            camera_display = camera_ax.imshow(image)

        else:

            camera_display.set_data(image)

        camera_fig.canvas.draw_idle()
        camera_fig.canvas.flush_events()

        plt.pause(0.01)

        # ANALYSE VIALS
        results = analyse_vial_positions(image)

        current_detection = (
            results["POS_1"]["colour"],
            results["POS_2"]["colour"],
            results["POS_3"]["colour"]
        )

        print(
            f"Detection: "
            f"{current_detection[0]}, "
            f"{current_detection[1]}, "
            f"{current_detection[2]}"
        )

        # Show current result above camera image
        camera_ax.set_title(
            "OV7670 - Identifying Vials\n"
            f"POS_1: {current_detection[0]}   "
            f"POS_2: {current_detection[1]}   "
            f"POS_3: {current_detection[2]}"
        )

        expected_colours = {
            "RED",
            "GREEN",
            "BLUE"
        }

        # Reject incorrect / startup readings
        if set(current_detection) != expected_colours:

            stable_count = 0
            last_detection = None
            continue

        # Same result as previous frame
        if current_detection == last_detection:

            stable_count += 1

        else:

            last_detection = current_detection
            stable_count = 1

        # COLOURS CONFIRMED
        if stable_count >= REQUIRED_STABLE_FRAMES:
            print("\n------------------------------")
            print("VIAL COLOURS CONFIRMED")
            print("------------------------------")
            print(f"Position 1: {current_detection[0]}")
            print(f"Position 2: {current_detection[1]}")
            print(f"Position 3: {current_detection[2]}")
            print("------------------------------")

            # Briefly show confirmed result
            camera_ax.set_title(
                "VIAL COLOURS CONFIRMED\n"
                f"POS_1: {current_detection[0]}   "
                f"POS_2: {current_detection[1]}   "
                f"POS_3: {current_detection[2]}"
            )

            camera_fig.canvas.draw_idle()
            camera_fig.canvas.flush_events()

            plt.pause(1)
            # Close camera window before arm starts
            plt.close(camera_fig)

            return current_detection


# IDENTIFY VIALS AND BUILD SORTING SEQUENCE
if send_to_arduino:
    detected_colours = identify_vials(arduino)
    sorting_plan = create_sorting_plan(detected_colours)

    print("\nSORTING PLAN")
    print("------------------------------")

    for item in sorting_plan:
        if item["action"] == "LEAVE":
            print(
                f"{item['position']}: "
                f"{item['colour']} "
                f"-> LEAVE IN PLACE"
            )

        else:
            print(
                f"{item['position']}: "
                f"{item['colour']} "
                f"-> MOVE {item['action']}"
            )

    print("------------------------------\n")

    waypoints = build_sorting_waypoints(sorting_plan)

    print("Building arm motion...")

    start_build_time = time.monotonic()
    motion = build_motion(waypoints)

    print(
        f"Motion built in "
        f"{time.monotonic() - start_build_time:.2f} seconds"
    )

    print(
        f"Total motion frames: {len(motion)}"
    )

    print(
        f"Planned animation duration: "
        f"{len(motion) * ani_interval / 1000:.1f} seconds"
    )






# ------------------------------------------
# 3D PLOT
# ------------------------------------------
# Command-line options for view angle
parser = argparse.ArgumentParser(description="3D robot arm visualiser")
parser.add_argument("--elev", type=float, default=30.0, help="Elevation angle in degrees")
parser.add_argument("--azim", type=float, default=135.0, help="Azimuth angle in degrees")
args = parser.parse_args()

# Plotting
fig = plt.figure()
fig.set_size_inches(9, 9)
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=args.elev, azim=args.azim)

ax.set_title("3-Axis Robot Arm Visualisation")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

max_reach = base_height + L1 + L2 + L3

ax.set_xlim(-max_reach, max_reach)
ax.set_ylim(-max_reach, max_reach)
ax.set_zlim(0, max_reach)

# Separate arm sections
base_section, = ax.plot([], [], [], linewidth=8, marker="o", markersize=8, color="black")
upper_arm_section, = ax.plot([], [], [], linewidth=6, marker="o", markersize=8, color="blue")
forearm_section, = ax.plot([], [], [], linewidth=4, marker="o", markersize=8, color="green")
wrist_section, = ax.plot([], [], [], linewidth=2, marker="o", markersize=8, color="red")

# Gripper tip marker
tip_marker, = ax.plot([], [], [], marker="x", markersize=12, color="purple")

# Text display for angles
angle_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)






def update(frame):
    position, gripper_command = motion[frame]
    x, y, z, phi = position
    theta1, theta2, theta3, theta4 = inverse_kinematics(x, y, z, phi)
    vert_coords = forward_kinematics(theta1, theta2, theta3, theta4)
    servo_angles = sim_to_real_angle(theta1, theta2, theta3, theta4)

    check_joint_limits(
        servo_angles[0],
        servo_angles[1],
        servo_angles[2],
        servo_angles[3]
    )

    send_servo_command(servo_angles, gripper_command)

    # Individual joint positions
    base_coords = vert_coords[0]
    shoulder_coords = vert_coords[1]
    elbow_coords = vert_coords[2]
    wrist_coords = vert_coords[3]
    tip_coords = vert_coords[4]

    # Base section: base to shoulder
    base_section.set_data(
        [base_coords[0], shoulder_coords[0]],
        [base_coords[1], shoulder_coords[1]])
    base_section.set_3d_properties(
        [base_coords[2], shoulder_coords[2]])

    # Upper arm section: shoulder to elbow
    upper_arm_section.set_data(
        [shoulder_coords[0], elbow_coords[0]],
        [shoulder_coords[1], elbow_coords[1]])
    upper_arm_section.set_3d_properties(
        [shoulder_coords[2], elbow_coords[2]])

    # Forearm section: elbow to wrist
    forearm_section.set_data(
        [elbow_coords[0], wrist_coords[0]],
        [elbow_coords[1], wrist_coords[1]])
    forearm_section.set_3d_properties(
        [elbow_coords[2], wrist_coords[2]])

    # Wrist section: wrist to gripper tip
    wrist_section.set_data(
        [wrist_coords[0], tip_coords[0]],
        [wrist_coords[1], tip_coords[1]])
    wrist_section.set_3d_properties(
        [wrist_coords[2], tip_coords[2]])

    # Gripper tip marker
    tip_marker.set_data(
        [tip_coords[0]],
        [tip_coords[1]])
    tip_marker.set_3d_properties(
        [tip_coords[2]])

    angle_text.set_text(
        f"Base rotation: {theta1:.1f}°\n"
        f"Shoulder: {theta2:.1f}°\n"
        f"Elbow: {theta3:.1f}°\n"
        f"Wrist: {theta4:.1f}°\n"
        f"Gripper tip: "
        f"({tip_coords[0]:.1f}, "
        f"{tip_coords[1]:.1f}, "
        f"{tip_coords[2]:.1f})"
    )

    return (
        base_section,
        upper_arm_section,
        forearm_section,
        wrist_section,
        tip_marker,
        angle_text
    )

print("starting animation")
animation = FuncAnimation(
    fig,
    update,
    frames=len(motion),
    interval=ani_interval,
    blit=False,
    repeat=False
)


try:
    plt.show()

finally:
    return_to_start()

    if arduino is not None:
        time.sleep(0.5)
        arduino.close()

    print("Program finished.")