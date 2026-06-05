import math
from controller import Camera, Keyboard
from vehicle import Driver

# ── Simulation timestep ───────────────────────────────────────────────────────
TIME_STEP = 50

# ── Autodrive PID constants (unchanged from working basal code) ───────────────
KP = 0.25
KI = 0.006
KD = 2

FILTER_SIZE = 3
UNKNOWN     = 99999.99

# ── Manual control ramp rates (per TIME_STEP tick = per 50 ms) ───────────────
# Speed: target changes by 1.0 km/h per keypress; actual cruising speed ramps
# toward the target at 1.0 km/h per tick → smooth ~1–2 s 0-to-50 ramp.
SPEED_KEY_STEP = 1.0      # km/h added to target per key event
SPEED_RAMP     = 1.0      # km/h the actual setpoint moves per 50 ms tick

# Steering: target angle changes by 0.005 rad per keypress; actual angle ramps
# toward that target at 0.005 rad per tick → very gentle, no shake/topple.
STEER_KEY_STEP = 0.005    # rad added to steer target per key event
STEER_RAMP     = 0.005    # rad the actual steering moves per 50 ms tick

SPEED_MAX      = 120.0    # km/h hard ceiling
SPEED_MIN_D    =   0.0    # km/h floor in Drive gear  (cannot go negative)
SPEED_MIN_R    = -30.0    # km/h floor in Reverse gear
SPEED_RETURN   =   0.2    # km/h per tick that autodrive target creeps back to pre-A speed
STEER_MAX      =   0.5    # rad (matches Webots physical limit)

# ── Curve management ──────────────────────────────────────────────────────────
# When the detected yellow-line angle exceeds CURVE_THRESHOLD the car is in a
# sharp turn. Speed is clamped to CURVE_SPEED_MAX and the steering ramp is
# tightened so corrections reach the wheels faster.
CURVE_THRESHOLD  = 0.06   # rad — above this = sharp curve detected
CURVE_SPEED_MAX  = 25.0   # km/h — speed ceiling while in a curve
CURVE_STEER_RAMP = 0.02   # rad/tick — faster ramp on curves for quicker correction
# PID output is divided by this factor at high speed to prevent overcorrection.
# At 50 km/h: factor≈1.25, at 80 km/h: factor≈2.0 — gentle damping.
SPEED_STEER_DAMP = 40.0   # km/h reference — higher = less damping effect

# ── Obstacle / emergency braking thresholds ───────────────────────────────────
# Time-To-Collision (TTC) based — automatically scales with speed.
# TTC = distance / speed. At 50 km/h and 10m: TTC = 0.72s → emergency.
# At 20 km/h and 10m: TTC = 1.8s → only caution.
TTC_EMERGENCY   =  3.0    # seconds — full brake if TTC below this
TTC_CAUTION     =  6.0    # seconds — progressive slow-down zone
MIN_SPEED_MPS   =  1.0    # m/s — below this speed TTC is unreliable, use fixed dist
EMERGENCY_DIST  =  3.0    # metres — fixed fallback when speed is very low
CAUTION_DIST    = 10.0    # metres — fixed fallback when speed is very low
FRONT_HALF_AREA =   5     # lidar beams either side of centre

# ── PID state (mimicking C static locals) ────────────────────────────────────
_pid_old_value = 0.0
_pid_integral  = 0.0
PID_need_reset = False

# ── filter_angle state ────────────────────────────────────────────────────────
_filter_first_call = True
_filter_old_value  = [0.0] * FILTER_SIZE


# ─────────────────────────────────────────────────────────────────────────────
# Helper functions (all identical to basal code — do NOT modify)
# ─────────────────────────────────────────────────────────────────────────────

def filter_angle(new_value):
    global _filter_first_call, _filter_old_value
    if _filter_first_call or new_value == UNKNOWN:
        _filter_first_call = False
        _filter_old_value  = [0.0] * FILTER_SIZE
    else:
        for i in range(FILTER_SIZE - 1):
            _filter_old_value[i] = _filter_old_value[i + 1]
    if new_value == UNKNOWN:
        return UNKNOWN
    _filter_old_value[FILTER_SIZE - 1] = new_value
    return sum(_filter_old_value) / FILTER_SIZE


def process_camera_image(image, camera_width, camera_height, camera_fov):
    if image is None:
        return UNKNOWN
    REF_R, REF_G, REF_B = 203, 187, 95
    sumx        = 0
    pixel_count = 0
    for y in range(camera_height):
        for x in range(camera_width):
            r    = Camera.imageGetRed(image,   camera_width, x, y)
            g    = Camera.imageGetGreen(image, camera_width, x, y)
            b    = Camera.imageGetBlue(image,  camera_width, x, y)
            diff = abs(int(r) - REF_R) + abs(int(g) - REF_G) + abs(int(b) - REF_B)
            if diff < 30:
                sumx        += x
                pixel_count += 1
    if pixel_count == 0:
        return UNKNOWN
    return ((sumx / pixel_count / camera_width) - 0.5) * camera_fov


def process_sick_data(sick, sick_width, sick_fov):
    HALF_AREA = 20
    sick_data = sick.getRangeImage()
    if sick_data is None or len(sick_data) == 0:
        return UNKNOWN, 0.0
    actual_width    = len(sick_data)
    centre          = actual_width // 2
    half            = min(HALF_AREA, centre)
    sumx            = 0
    collision_count = 0
    obstacle_dist   = 0.0
    for x in range(centre - half, centre + half):
        range_val = sick_data[x]
        if range_val > 0 and range_val < 20.0:
            sumx            += x
            collision_count += 1
            obstacle_dist   += range_val
    if collision_count == 0:
        return UNKNOWN, 0.0
    obstacle_dist /= collision_count
    angle = ((sumx / collision_count / actual_width) - 0.5) * sick_fov
    return angle, obstacle_dist


_lidar_debug_printed = False

def check_front_obstacle(sick, sick_width):
    """
    Returns (emergency, caution, min_dist) where emergency/caution are
    placeholder booleans — actual decision is TTC-based in the main loop.
    """
    global _lidar_debug_printed

    sick_data = sick.getRangeImage()
    if sick_data is None or len(sick_data) == 0:
        return False, False, float('inf')

    actual_width = len(sick_data)
    centre       = actual_width // 2
    half         = min(FRONT_HALF_AREA, centre)

    # Keep printing until real finite non-zero values appear
    if not _lidar_debug_printed:
        sample = [round(sick_data[i], 2) for i in range(max(0, centre-5), min(actual_width, centre+5))]
        print(f"Lidar centre samples: {sample}")
        if any(0 < v < 1e6 and not math.isinf(v) for v in sample):
            _lidar_debug_printed = True

    min_dist = float('inf')
    for x in range(centre - half, centre + half):
        val = sick_data[x]
        if val > 0 and not math.isinf(val) and val < min_dist:
            min_dist = val

    # emergency/caution booleans are computed TTC-based in main loop
    # but we still return them for the process_sick_data compatibility
    emergency = min_dist < EMERGENCY_DIST
    caution   = min_dist < CAUTION_DIST
    return emergency, caution, min_dist


def applyPID(yellow_line_angle):
    global _pid_old_value, _pid_integral, PID_need_reset
    if PID_need_reset:
        _pid_old_value = yellow_line_angle
        _pid_integral  = 0.0
        PID_need_reset = False
    if math.copysign(1, yellow_line_angle) != math.copysign(1, _pid_old_value):
        _pid_integral = 0.0
    diff = yellow_line_angle - _pid_old_value
    if -30 < _pid_integral < 30:
        _pid_integral += yellow_line_angle
    _pid_old_value = yellow_line_angle
    return KP * yellow_line_angle + KI * _pid_integral + KD * diff


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global PID_need_reset

    driver          = Driver()
    basic_time_step = int(driver.getBasicTimeStep())

    keyboard = Keyboard()
    keyboard.enable(TIME_STEP)

    # ── Device discovery ──────────────────────────────────────────────────────
    has_camera                 = False
    enable_display             = False
    enable_collision_avoidance = False
    has_gps                    = False
    lidar_name                 = None   # will hold the actual name found

    # Common lidar/distance sensor names used across Webots vehicle worlds
    LIDAR_NAMES = ["Sick LMS 291", "lidar", "sick", "Lidar", "SICK",
                   "distance sensor", "rangefinder", "LDS"]

    print("Discovered devices:")
    for i in range(driver.getNumberOfDevices()):
        device = driver.getDeviceByIndex(i)
        name   = device.getName()
        print(f"  [{i}] {name}")
        if   name == "camera":              has_camera                 = True
        elif name == "display":             enable_display             = True
        elif name == "gps":                 has_gps                    = True
        elif name in LIDAR_NAMES:
            enable_collision_avoidance = True
            lidar_name                 = name
    print(f"Camera: {has_camera} | Lidar: {enable_collision_avoidance} "
          f"({lidar_name}) | GPS: {has_gps} | Display: {enable_display}\n")

    # ── Hardware bindings (identical to basal code) ───────────────────────────
    camera        = None
    camera_width  = -1
    camera_height = -1
    camera_fov    = -1.0
    if has_camera:
        camera        = driver.getDevice("camera")
        camera.enable(TIME_STEP)
        camera_width  = camera.getWidth()
        camera_height = camera.getHeight()
        camera_fov    = camera.getFov()

    sick       = None
    sick_width = -1
    sick_fov   = -1.0
    if enable_collision_avoidance and lidar_name:
        sick       = driver.getDevice(lidar_name)
        sick.enable(TIME_STEP)
        sick_width = sick.getHorizontalResolution()
        sick_fov   = sick.getFov()
        print(f"Lidar ready: width={sick_width}, fov={sick_fov:.2f}, maxRange={sick.getMaxRange()}")

    gps        = None
    gps_coords = [0.0, 0.0, 0.0]
    gps_speed  = 0.0
    if has_gps:
        gps = driver.getDevice("gps")
        gps.enable(TIME_STEP)

    display           = None
    speedometer_image = None
    if enable_display:
        display           = driver.getDevice("display")
        speedometer_image = display.imageLoad("speedometer.png")

    # ── Vehicle accessories (identical to basal code) ─────────────────────────
    if has_camera:
        driver.setCruisingSpeed(50.0)

    driver.setHazardFlashers(True)
    driver.setDippedBeams(True)
    driver.setAntifogLights(True)
    driver.setWiperMode(Driver.SLOW)

    # ── State variables ───────────────────────────────────────────────────────
    # autodrive: True  → camera/PID controls steering
    #            False → keyboard controls both speed and steering
    autodrive = True

    # Gear: 'A' = autodrive, 'D' = drive (forward), 'R' = reverse
    gear = 'A'

    # Actual values being sent to the driver this tick
    cruising_speed  = 50.0 if has_camera else 0.0
    steering_angle  = 0.0

    # Targets that keyboard nudges; ramp logic closes the gap each tick
    target_speed    = cruising_speed
    target_steering = 0.0

    # Speed the driver was doing just before A was pressed; autodrive creeps back to this
    pre_autodrive_speed = target_speed

    # Speed before caution/emergency intervention — restored when obstacle clears
    pre_caution_speed   = target_speed

    # Counts consecutive ticks where the yellow line is not visible (intersection crossing etc.)
    # Steering is only altered after LINE_LOST_PATIENCE ticks of continuous loss.
    lost_line_ticks       = 0
    LINE_LOST_PATIENCE    = 20   # ticks (~1 second at 50ms) before reacting to lost line

    # Curve speed management
    in_curve        = False
    pre_curve_speed = target_speed

    print("\n--- Webots Python Controller ---")
    print("[UP/DOWN]    Adjust speed smoothly")
    print("[LEFT/RIGHT] Steer smoothly  (switches to manual mode)")
    print("[A]          Autodrive lane-following  (enters at 20 km/h, resumes prior speed)")
    print("[R]          Reverse gear  (DOWN then accelerates backwards)")
    print("[D]          Drive gear    (back to forward-only)")
    print("[SPACE]      Brake")
    print("Emergency braking: TTC-based (scales with speed automatically)")
    print("------------------------------------------------\n")

    loop_count        = 0
    update_ratio      = max(1, int(TIME_STEP / basic_time_step))
    last_printed_dist = None   # throttle obstacle status prints — only print on change

    # ─────────────────────────────────────────────────────────────────────────
    # Main loop
    # ─────────────────────────────────────────────────────────────────────────
    while driver.step() != -1:

        # ── 1. Emergency brake check (every tick, overrides everything) ─────────
        # Uses Time-To-Collision (TTC = dist / speed) so braking scales with speed.
        # At high speed the caution/emergency zones are much larger automatically.
        # Reverse excluded — obstacle logic doesn't apply behind the car.
        emergency_brake = False
        if enable_collision_avoidance and sick is not None and cruising_speed > 0:
            emergency, caution, front_dist = check_front_obstacle(sick, sick_width)

            # Convert current speed to m/s for TTC calculation
            speed_mps = cruising_speed / 3.6

            if speed_mps > MIN_SPEED_MPS:
                # Speed-adaptive: compute TTC
                ttc = front_dist / speed_mps
                is_emergency = ttc < TTC_EMERGENCY
                is_caution   = ttc < TTC_CAUTION
            else:
                # Low/zero speed: fall back to fixed distance thresholds
                is_emergency = front_dist < EMERGENCY_DIST
                is_caution   = front_dist < CAUTION_DIST

            if is_emergency:
                driver.setBrakeIntensity(1.0)
                if last_printed_dist is None:        # first entry — save ceiling once
                    pre_caution_speed = pre_autodrive_speed
                target_speed        = 0.0
                cruising_speed      = 0.0            # stop cruise controller immediately
                driver.setCruisingSpeed(0.0)         # send zero directly — don't wait for ramp
                emergency_brake     = True
                last_printed_dist   = front_dist

            elif is_caution:
                if speed_mps > MIN_SPEED_MPS:
                    ttc_factor = (ttc - TTC_EMERGENCY) / (TTC_CAUTION - TTC_EMERGENCY)
                else:
                    ttc_factor = (front_dist - EMERGENCY_DIST) / (CAUTION_DIST - EMERGENCY_DIST)
                safe_speed = cruising_speed * max(0.0, min(1.0, ttc_factor))
                if last_printed_dist is None:        # first entry — save ceiling once
                    pre_caution_speed = pre_autodrive_speed
                if target_speed > safe_speed:
                    target_speed = safe_speed
                # Do NOT touch pre_autodrive_speed — restore will fix it on clear
                last_printed_dist = front_dist

            else:
                # Obstacle gone — restore ceiling to what it was before any intervention
                if last_printed_dist is not None:
                    pre_autodrive_speed = pre_caution_speed
                    last_printed_dist   = None

        # ── 2. Read keyboard ──────────────────────────────────────────────────
        key = keyboard.getKey()

        # ── 3. Speed keys ─────────────────────────────────────────────────────
        # Floor depends on gear: Reverse allows negative, Drive stays >= 0
        speed_floor = SPEED_MIN_R if gear == 'R' else SPEED_MIN_D

        if key == Keyboard.UP:
            if gear == 'R':
                # In reverse, UP decelerates toward 0 — cannot go positive
                target_speed = min(target_speed + SPEED_KEY_STEP, 0.0)
            else:
                target_speed = min(target_speed + SPEED_KEY_STEP, SPEED_MAX)
            if autodrive:
                pre_autodrive_speed = target_speed  # raise restore ceiling
                if in_curve:
                    pre_curve_speed = target_speed  # keep curve restore in sync too

        elif key == Keyboard.DOWN:
            # DOWN decelerates; in Drive floors at 0, in Reverse floors at -30
            target_speed = max(target_speed - SPEED_KEY_STEP, speed_floor)
            if autodrive:
                pre_autodrive_speed = target_speed  # lower the restore ceiling too

        # ── 4. Steering keys ──────────────────────────────────────────────────
        elif key == Keyboard.RIGHT:
            if autodrive:
                autodrive = False
                gear      = 'D'
                target_steering = steering_angle
                print("Gear: DRIVE  (press A for autodrive)")
            target_steering = min(target_steering + STEER_KEY_STEP, STEER_MAX)

        elif key == Keyboard.LEFT:
            if autodrive:
                autodrive = False
                gear      = 'D'
                target_steering = steering_angle
                print("Gear: DRIVE  (press A for autodrive)")
            target_steering = max(target_steering - STEER_KEY_STEP, -STEER_MAX)

        else:
            # No steering key held — centre wheels in manual mode
            if not autodrive:
                target_steering = 0.0

        # ── 5. Mode and gear keys (plain 'if' so they always fire) ───────────

        if key == ord('A'):
            if gear != 'A':                         # only act on actual change
                pre_autodrive_speed = target_speed
                autodrive           = True
                target_speed        = 20.0
                target_steering     = 0.0
                gear                = 'A'
                print("Gear: AUTODRIVE  (20 km/h → restoring to prior speed gradually)")

        if key == ord('R'):
            if gear != 'R':                         # only act on actual change
                autodrive    = False
                gear         = 'R'
                target_speed = min(target_speed, 0.0)
                print("Gear: REVERSE")

        if key == ord('D'):
            if gear != 'D':                         # only act on actual change
                autodrive    = False
                gear         = 'D'
                target_speed = max(target_speed, 0.0)
                print("Gear: DRIVE")

        # ── 6. Spacebar = full brake (skipped if emergency brake already active) ─
        if not emergency_brake:
            if key == ord(' '):
                driver.setBrakeIntensity(1.0)
                target_speed = max(target_speed - SPEED_RAMP * 4, 0.0)
                if autodrive:
                    pre_autodrive_speed = target_speed
            else:
                driver.setBrakeIntensity(0.0)

        # ── 7. Autodrive speed restore ────────────────────────────────────────
        if (autodrive
                and not emergency_brake
                and target_speed < pre_autodrive_speed
                and key != Keyboard.DOWN
                and key != ord(' ')):
            target_speed = min(target_speed + SPEED_RETURN, pre_autodrive_speed)

        # ── 8. Ramp actual cruising speed toward target every tick ────────────
        if cruising_speed < target_speed:
            cruising_speed = min(cruising_speed + SPEED_RAMP, target_speed)
        elif cruising_speed > target_speed:
            cruising_speed = max(cruising_speed - SPEED_RAMP, target_speed)
        driver.setCruisingSpeed(cruising_speed)

        # ── 9. Sensor + steering update every TIME_STEP ms ───────────────────
        if loop_count % update_ratio == 0:

            if autodrive and has_camera and camera is not None:
                # ── Autodrive path (identical to basal code) ──────────────────
                raw_image = camera.getImage()

                yellow_line_angle = filter_angle(
                    process_camera_image(raw_image, camera_width, camera_height, camera_fov)
                )

                obstacle_angle = UNKNOWN
                obstacle_dist  = 0.0
                if enable_collision_avoidance and sick is not None:
                    obstacle_angle, obstacle_dist = process_sick_data(sick, sick_width, sick_fov)

                if enable_collision_avoidance and obstacle_angle != UNKNOWN and obstacle_dist > 0:
                    if key != ord(' '):
                        driver.setBrakeIntensity(0.0)
                    obstacle_steering = steering_angle
                    if 0.0 < obstacle_angle < 0.4:
                        obstacle_steering = steering_angle + (obstacle_angle - 0.25) / obstacle_dist
                    elif obstacle_angle > -0.4:
                        obstacle_steering = steering_angle + (obstacle_angle + 0.25) / obstacle_dist
                    steer = steering_angle
                    if yellow_line_angle != UNKNOWN:
                        line_following_steering = applyPID(yellow_line_angle)
                        if obstacle_steering > 0 and line_following_steering > 0:
                            steer = max(obstacle_steering, line_following_steering)
                        elif obstacle_steering < 0 and line_following_steering < 0:
                            steer = min(obstacle_steering, line_following_steering)
                    else:
                        PID_need_reset = True
                    target_steering  = steer
                    lost_line_ticks  = 0

                elif yellow_line_angle != UNKNOWN:
                    # Line visible — curve-adaptive PID following
                    if key != ord(' '):
                        driver.setBrakeIntensity(0.0)

                    curve_intensity = abs(yellow_line_angle)
                    is_sharp_curve  = curve_intensity > CURVE_THRESHOLD

                    if is_sharp_curve and not in_curve:
                        # Entering a curve — save user's intended cruise speed
                        in_curve        = True
                        pre_curve_speed = pre_autodrive_speed  # save ceiling, not current target

                    if in_curve and not is_sharp_curve:
                        # Exiting curve — restore ceiling to user's intended speed
                        # SPEED_RETURN ramp climbs back gradually from wherever target_speed is
                        in_curve            = False
                        pre_autodrive_speed = pre_curve_speed

                    if is_sharp_curve:
                        # On a curve: clamp speed, apply damped PID
                        if target_speed > CURVE_SPEED_MAX:
                            target_speed = CURVE_SPEED_MAX

                    # Damp PID output at high speed to prevent overcorrection
                    speed_damp_factor = max(1.0, cruising_speed / SPEED_STEER_DAMP)
                    target_steering   = applyPID(yellow_line_angle) / speed_damp_factor
                    lost_line_ticks   = 0

                else:
                    # Line not visible (intersection, shadow, gap in markings)
                    lost_line_ticks += 1
                    if in_curve:
                        # Lost line mid-curve — exit curve mode so speed isn't
                        # permanently clamped if we re-acquire on a straight
                        in_curve = False
                    if lost_line_ticks <= LINE_LOST_PATIENCE:
                        # Hold last steering angle and speed — coast straight through
                        if key != ord(' '):
                            driver.setBrakeIntensity(0.0)
                        # target_steering unchanged — car keeps its current heading
                    else:
                        # Line genuinely lost for too long — slow gently, straighten gradually
                        if key != ord(' '):
                            driver.setBrakeIntensity(0.2)
                        PID_need_reset  = True
                        target_steering = 0.0

            # ── Ramp actual steering toward target (manual AND autodrive) ─────
            # Use faster ramp in autodrive curve to get corrections to wheels quickly.
            # In manual or straight autodrive, use the gentle base ramp.
            if autodrive and in_curve:
                active_steer_ramp = CURVE_STEER_RAMP
            else:
                active_steer_ramp = STEER_RAMP

            if steering_angle < target_steering:
                steering_angle = min(steering_angle + active_steer_ramp, target_steering)
            elif steering_angle > target_steering:
                steering_angle = max(steering_angle - active_steer_ramp, target_steering)
            # Hard clamp before sending
            clamped = max(-STEER_MAX, min(STEER_MAX, steering_angle))
            driver.setSteeringAngle(clamped)

            # ── GPS update (identical to basal code) ──────────────────────────
            if has_gps and gps is not None:
                coords    = gps.getValues()
                speed_ms  = gps.getSpeed()
                gps_speed = speed_ms * 3.6
                gps_coords[0], gps_coords[1], gps_coords[2] = coords[0], coords[1], coords[2]

            # ── Speedometer (identical to basal code) ─────────────────────────
            if enable_display and display is not None and speedometer_image is not None:
                NEEDLE_LENGTH = 50.0
                display.imagePaste(speedometer_image, 0, 0, False)
                current_speed = driver.getCurrentSpeed()
                if math.isnan(current_speed):
                    current_speed = 0.0
                alpha = (abs(current_speed) / 260.0) * 3.72 - 0.27
                x     = int(-NEEDLE_LENGTH * math.cos(alpha))
                y     = int(-NEEDLE_LENGTH * math.sin(alpha))
                display.setColor(0xFFFFFF)
                display.drawLine(100, 95, 100 + x, 95 + y)
                if has_gps:
                    display.setColor(0x000000)
                    display.drawText(f"GPS coords: {gps_coords[0]:.1f} {gps_coords[2]:.1f}", 10, 130)
                    display.drawText(f"GPS speed:  {gps_speed:.1f}", 10, 140)

        loop_count += 1


if __name__ == "__main__":
    main()
