from base64 import b16encode
import can
import csv
import math
import numpy as np
import os
import time

# Set up can bus
os.system('sudo ip link set can0 up type can bitrate 1000000')
os.system('sudo ifconfig can0 txqueuelen 1000')
bus = can.interface.Bus(channel = 'can0', interface = 'socketcan')

# Motor parameters
rotor_counts_per_rev = 8192
reduction = 36

# Controller parameters
Kp = 2500
Kd = 50
prev_rotor_count = 0
nr_shaft_rotations = 0

# Open file to write data
filename = 'data.csv'
with open(filename, 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    header = [
	      "timestamp",
              "id",
              "extended_id",
              "RX",
              "remote_frame",
              "error_frame",
              "dlc",
              "rotor_count_hex",
              "rotor_speed_hex",
              "current_hex"
             ]
    csvwriter.writerow(header)
    start = time.time()
    end = start + 5

    msg = bus.recv()
    shaft_angle_zero = 2 * np.pi * int.from_bytes(msg.data[:2], byteorder = 'big') / (rotor_counts_per_rev * reduction)

# Listen for messages and generate current commands
    while time.time() < end:
        msg = bus.recv()
        rotor_count = int.from_bytes(msg.data[:2], byteorder = 'big')
        rotor_rpm = int.from_bytes(msg.data[2:4], byteorder = 'big')
        torque_current_A = int.from_bytes(msg.data[4:6], byteorder = 'big')
        print(str(msg.dlc), msg.data)
        print(f"rotor_count: {rotor_count}, rotor_rpm: {rotor_rpm}, torque_current: {torque_current_A}")
        row = [
               repr(msg.timestamp),
               hex(msg.arbitration_id),
               "1" if msg.is_extended_id else "0",
               "1" if msg.is_rx else "0",
               "1" if msg.is_remote_frame else "0",
               "1" if msg.is_error_frame else "0",
               msg.dlc,
               b16encode(msg.data[0:2]).decode("utf8"),
               b16encode(msg.data[2:4]).decode("utf8"),
               b16encode(msg.data[4:6]).decode("utf8")
              ]
        csvwriter.writerow(row)

# Determine sinusoidal target
        target = 0.5 * np.sin(1000 * (time.time() -  start) / 500) - shaft_angle_zero

# Determine current shaft angle in radians
        rotor_count = int.from_bytes(msg.data[0:2],byteorder = 'big')
        delta = rotor_count - prev_rotor_count
        if delta < -1 * 0.5 * rotor_counts_per_rev:
            nr_shaft_rotations = nr_shaft_rotations + 1
        elif delta > 0.5 * rotor_counts_per_rev:
            nr_shaft_rotations = nr_shaft_rotations - 1
        if nr_shaft_rotations == 37 or nr_shaft_rotations == -37:
            nr_shaft_rotations = 0
        corr_count = rotor_count + nr_shaft_rotations * rotor_counts_per_rev
        prev_rotor_count = rotor_count
        shaft_angle = 2 * np.pi * corr_count / (rotor_counts_per_rev * reduction)

# Determine current speed in radians per second
        speed = 2 * np.pi * int.from_bytes(msg.data[2:4],byteorder = 'big') / (60 * reduction)

# Determine current using controller and convert to hex
        current =  Kp * (target - shaft_angle) + Kd * -1 * speed
        print(shaft_angle_zero, shaft_angle, target, math.floor(current))
        if abs(current) > 4000:
           current = 0
        if current >= 0:
            bytes = (math.floor(current)).to_bytes(2, byteorder = 'big')
        else:
            bytes = (math.floor(current)).to_bytes(2, byteorder = 'big', signed = True)
#       print(b16encode(bytes).decode("utf8"))
#       if time.time() < start + 15:
#           command  = can.Message(arbitration_id = 0x200, data = [0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id = False)
#       else:
#           command  = can.Message(arbitration_id = 0x200, data = [0xFE, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id = False)
        command  = can.Message(arbitration_id = 0x200, data = [bytes[0], bytes[1], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id = False)
        bus.send(command)

# Sleep
#       time.sleep(0.001)

# Close can bus
os.system('sudo ifconfig can0 down')
