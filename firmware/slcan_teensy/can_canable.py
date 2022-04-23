import can

bus = can.interface.Bus(bustype = 'slcan', channel = 'COM7', bitrate=500000)

msg = can.Message(arbitration_id=9, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=True)
received = False
while received == False:
    msg = bus.recv()
    print(msg)