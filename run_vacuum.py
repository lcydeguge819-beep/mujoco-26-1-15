import mujoco
from mujoco import viewer
import numpy as np

def main():
    print("Loading model...")
    model = mujoco.MjModel.from_xml_path("trae-26-1-19.xml")
    data = mujoco.MjData(model)

    # Correct API calls for MuJoCo 3.x+
    print("Initializing sensors and actuators...")
    act_left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "vacuum_motor_left")
    act_right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "vacuum_motor_right")

    s_front = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_front")
    s_front_left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_front_left")
    s_front_right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_front_right")
    s_back = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_back")
    s_left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_left")
    s_right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_right")

    # Check if IDs are valid (-1 means not found)
    if any(id == -1 for id in [act_left, act_right, s_front, s_front_left, s_front_right, s_back, s_left, s_right]):
        print("Error: Could not find one or more actuators/sensors in the XML.")
        print(f"IDs: L={act_left}, R={act_right}, F={s_front}, FL={s_front_left}, FR={s_front_right}, B={s_back}, LF={s_left}, RT={s_right}")
        return

    adr_front = model.sensor_adr[s_front]
    adr_front_left = model.sensor_adr[s_front_left]
    adr_front_right = model.sensor_adr[s_front_right]
    adr_back = model.sensor_adr[s_back]
    adr_left = model.sensor_adr[s_left]
    adr_right = model.sensor_adr[s_right]

    def read_range(adr):
        # Rangefinder data is a single scalar
        return data.sensordata[adr]

    # Manual Control using Keyboard
    import keyboard

    def control_step():
        front = read_range(adr_front)
        # We can still read sensors for display, but control is manual
        
        base_speed = 0.1  # Adjusted to 0.1 per user request
        turn_speed = 0.08  # Reduced turn speed (was 1.2)
        
        ctrl_l = 0.0
        ctrl_r = 0.0
        
        # Check keys
        # Note: keyboard.is_pressed requires access. If it fails, we might need a fallback or admin.
        # But usually it works for reading state.
        
        if keyboard.is_pressed('up'):
            ctrl_l += base_speed
            ctrl_r += base_speed
        elif keyboard.is_pressed('down'):
            ctrl_l -= base_speed
            ctrl_r -= base_speed
            
        if keyboard.is_pressed('left'):
            ctrl_l -= turn_speed
            ctrl_r += turn_speed
        elif keyboard.is_pressed('right'):
            ctrl_l += turn_speed
            ctrl_r -= turn_speed
            
        data.ctrl[act_left] = ctrl_l
        data.ctrl[act_right] = ctrl_r

    print("MuJoCo model loaded successfully.")
    print("=========================================")
    print("   MANUAL CONTROL MODE ACTIVATED")
    print("   Use ARROW KEYS to drive the robot")
    print("   UP:    Forward")
    print("   DOWN:  Backward")
    print("   LEFT:  Turn Left")
    print("   RIGHT: Turn Right")
    print("   (Ensure this window or the console has focus if inputs lag)")
    print("=========================================")
    print("Press Space to toggle pause in the viewer.")

    # Use launch_passive for better compatibility
    with viewer.launch_passive(model, data) as v:
        print("Viewer launched. Waiting for input...")
        step_count = 0
        while v.is_running():
            step_count += 1
            control_step()
            mujoco.mj_step(model, data)
            v.sync()
            
            if step_count % 200 == 0:
                # Robot is the 3rd free joint (block_1, block_2, vacuum_base)
                # Each free joint has 7 qpos values (x,y,z, rw,rx,ry,rz)
                # Index of vacuum_base qpos is 14
                robot_z = data.qpos[16]
                # print(f"Step {step_count}: Z={robot_z:.3f}, Front={read_range(adr_front):.2f}, L_Motor={data.ctrl[act_left]:.2f}, R_Motor={data.ctrl[act_right]:.2f}")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()
        input("Press Enter to exit...")
