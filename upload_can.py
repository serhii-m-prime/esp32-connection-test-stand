import serial
import time
import struct
from SCons.Script import Import

Import("env")

def custom_can_uploader(target, source, env):
    port = env.GetProjectOption("upload_port", "COM3")
    firmware_path = str(source[0].get_abspath())
    env_name = env.subst("$PIOENV")

    if env_name == "slave_env":
        target_node_id = 1
    elif env_name == "slave_motion":
        target_node_id = 2
    else:
        target_node_id = 0

    if target_node_id == 0:
        print(f"[CAN UPLOADER] Error: Unknown node destination for environment '{env_name}'")
        return 1

    ser = serial.Serial(port, 115200, timeout=1)

    def send_can_frame(can_id, data):
        dlc = len(data)
        padded_data = data + b'\x00' * (8 - dlc)
        frame = struct.pack('>BBIB8s', 0xAA, 0x55, can_id, dlc, padded_data)
        ser.write(frame)
        time.sleep(0.002)

    print(f"[CAN UPLOADER] Init flashing sequence for target Node ID: {target_node_id} via {port}")
    
    send_can_frame(0x0F0, bytes([target_node_id, 1]))
    time.sleep(0.5)

    print(f"[CAN UPLOADER] Streaming binary file: {firmware_path}")
    
    with open(firmware_path, 'rb') as f:
        while True:
            chunk = f.read(8)
            if not chunk:
                break
            send_can_frame(0x0F1, chunk)

    print("[CAN UPLOADER] Finalizing update transaction...")
    send_can_frame(0x0F0, bytes([target_node_id, 2]))

    print(f"[CAN UPLOADER] Target Node {target_node_id} flash write successful! Device will now reboot.")
    ser.close()
    return 0

env.Replace(UPLOADCMD=custom_can_uploader)