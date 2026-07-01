import can

bus = can.interface.Bus(
    channel='can0',
    bustype='socketcan'
)

while True:
    msg = bus.recv()

    if msg.arbitration_id == 0x07000001:

        data = msg.data

        # combine 2 bytes
        value = (data[0] << 8) | data[1]

        # unsigned -> signed
        if value >= 32768:
            value -= 65536

        print("Steering raw:", value)