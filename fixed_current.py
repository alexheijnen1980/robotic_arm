from base64 import b16encode
import can
import csv
import os
import time

# Set up can bus
os.system('sudo ip link set can0 up type can bitrate 1000000')
os.system('sudo ifconfig can0 txqueuelen 1000')
bus = can.interface.Bus(channel = 'can0', interface = 'socketcan')

# Motor parameters
counts_per_rev_shaft = 8192
reduction = 36

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
    end = time.time() + 30

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
        command  = can.Message(arbitration_id = 0x200, data = [0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id = False)
        bus.send(command)
# Sleep
#       time.sleep(0.001)

# Close can bus
os.system('sudo ifconfig can0 down')
