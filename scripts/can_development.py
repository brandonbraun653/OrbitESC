import can
from loguru import logger

# Test and see if the embedded code can print this out over the debug port

if __name__ == "__main__":
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=100000)
    msg = can.Message(arbitration_id=0x45, is_extended_id=False, data=[1, 2, 3, 4, 5])

    logger.info("Sending data")
    bus.send(msg=msg)
