import can
bus = can.interface.Bus(bustype = 'slcan', channel = 'COM7', bitrate=100000)
msg = can.Message(arbitration_id=0x9, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=True)
try:
    bus.send(msg)
    print("message sent")
except can.CanError:
    print("message not sent")