import can
# Candlelight firmware on Linux
#bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

# Stock slcan firmware on Linux
# bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)

# Stock slcan firmware on Windows
bus = can.interface.Bus(bustype='slcan', channel='COM7', bitrate=100000)

msg = can.Message(arbitration_id=0x00,
                  data=[0, 25, 0, 1, 3, 1, 4, 1],
                  is_extended_id=True)

try:
    bus.send(msg)
    print('Message sent on {}'.format(bus.channel_info))
except can.CanError:
    print('Message NOT sent')