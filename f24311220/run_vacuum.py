import mujoco
from mujoco import viewer
import numpy as np
import cv2
import time

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
        
    # Get Joint ID for Reset
    jnt_vacuum = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "vacuum_joint")
    qpos_adr = model.jnt_qposadr[jnt_vacuum]
    qvel_adr = model.jnt_dofadr[jnt_vacuum]

    adr_front = model.sensor_adr[s_front]
    adr_front_left = model.sensor_adr[s_front_left]
    adr_front_right = model.sensor_adr[s_front_right]
    adr_back = model.sensor_adr[s_back]
    adr_left = model.sensor_adr[s_left]
    adr_right = model.sensor_adr[s_right]

    # TV Video Setup
    video_path = r"C:\Users\lovary\Desktop\2222\1.mp4"
    cap = cv2.VideoCapture(video_path)
    
    tex_screen_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, "tex_screen")
    has_tv = (tex_screen_id != -1)
    
    if has_tv:
        tex_h = model.tex_height[tex_screen_id]
        tex_w = model.tex_width[tex_screen_id]
        tex_adr = model.tex_adr[tex_screen_id]
        tex_size = tex_w * tex_h * 3
        print(f"TV Screen Texture found: ID={tex_screen_id}, Size={tex_w}x{tex_h}")
    else:
        print("Warning: TV texture 'tex_screen' not found.")

    def read_range(adr):
        # Rangefinder data is a single scalar
        return data.sensordata[adr]

    # Manual Control using Keyboard
    import keyboard

    def control_step():
        front = read_range(adr_front)
        # We can still read sensors for display, but control is manual
        
        base_speed = 0.2  # Doubled from 0.1 per user request
        turn_speed = 0.16  # Doubled from 0.08 per user request
        
        ctrl_l = 0.0
        ctrl_r = 0.0
        
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
            
        # Reset Function (Press 'R')
        if keyboard.is_pressed('r'):
            # Reset Position: x=0, y=0, z=0.1
            data.qpos[qpos_adr] = 0
            data.qpos[qpos_adr+1] = 0
            data.qpos[qpos_adr+2] = 0.1
            # Reset Orientation: w=1, x=0, y=0, z=0
            data.qpos[qpos_adr+3] = 1
            data.qpos[qpos_adr+4] = 0
            data.qpos[qpos_adr+5] = 0
            data.qpos[qpos_adr+6] = 0
            # Reset Velocities
            for i in range(6):
                data.qvel[qvel_adr+i] = 0
            print("Robot Reset to Original Position")
            
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
    print("   Press 'R' to RESET robot position")
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

            # Update TV texture roughly every 30ms (approx 33fps) -> every 6 steps (5ms * 6 = 30ms)
            if has_tv and step_count % 6 == 0:
                ret, frame = cap.read()
                if not ret:
                    # Loop video
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = cap.read()
                
                if ret:
                    try:
                        frame = cv2.resize(frame, (tex_w, tex_h))
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        model.tex_rgb[tex_adr : tex_adr + tex_size] = frame.flatten()
                    except Exception as e:
                        pass # Ignore texture errors

            v.sync()
            
            if step_count % 200 == 0:
                # Debug output
                d_f = read_range(adr_front)
                # print(f"Step {step_count}: Front Dist={d_f:.2f}")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()
        input("Press Enter to exit...")
