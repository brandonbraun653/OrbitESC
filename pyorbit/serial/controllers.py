from __future__ import annotations

import struct
import time
from pyorbit.nanopb.system_config_pb2 import *
from typing import Optional
from loguru import logger
from pyorbit.serial.messages import AckNackPBMsg
from pyorbit.serial.parameters import ParameterId, ParameterType, ParamIOPBMsg, MessageEncoding
from pyorbit.serial.pipe import SerialPipePublisher


class ParameterController:
    """ Simple controller for interacting with the parameter registry off the OrbitESC debug port """

    def __init__(self, pipe: SerialPipePublisher):
        self._com_pipe = pipe

    def get(self, param: ParameterId) -> Optional[ParameterType]:
        """
        Requests the current value of a parameter
        Args:
            param: Which parameter to get

        Returns:
            Deserialized value
        """
        # Format and publish the request
        msg = ParamIOPBMsg()
        msg.sub_id = ParamIOSubId.GET
        msg.param_id = param

        # Push the request onto the wire
        start_time = time.time()
        rsp = self._com_pipe.write_and_wait(msg, 3.0)

        # Server denied the request for some reason
        if isinstance(rsp, AckNackPBMsg):
            logger.error(f"GET parameter {repr(param)} failed with status code: "
                         f"{repr(rsp.status_code)}")
            return None

        # Deserialize the data returned
        if isinstance(rsp, ParamIOPBMsg):
            logger.trace(f"Transaction completed in {time.time() - start_time:.2f} seconds for {repr(param)}")
            try:
                if rsp.param_type == MessageEncoding.STRING:
                    return rsp.data.decode('utf-8')
                elif rsp.param_type == MessageEncoding.FLOAT or rsp.param_type == MessageEncoding.DOUBLE:
                    return float(struct.unpack("<f", rsp.data)[0])
                elif rsp.param_type == MessageEncoding.UINT8 or rsp.param_type == MessageEncoding.UINT16 or \
                        rsp.param_type == MessageEncoding.UINT32:
                    return int.from_bytes(rsp.data, byteorder='little')
                elif rsp.param_type == MessageEncoding.BOOL:
                    assert len(rsp.data) == 1, f"Expected length 1 for bool type but got {len(rsp.data)}"
                    assert rsp.data[0] == 0 or rsp.data[0] == 1, f"Expected 0 or 1 for bool type but got {rsp.data[0]}"
                    return bool(rsp.data[0])
                else:
                    logger.warning(f"Don't know how to decode parameter type {rsp.param_type}")
                    return rsp.data
            except ValueError:
                logger.error(f"Failed to decode response: {msg}")
                return None

        logger.error("No response from server")
        return None

    def set(self, param: ParameterId, value: ParameterType) -> bool:
        """
        Sets the value of a parameter
        Args:
            param: Which parameter to set
            value: Value being assigned

        Returns:
            True if the operation succeeds, False if not
        """
        from pyorbit.app.parameters.util import parameter_encoding

        # Build the base of the message
        msg = ParamIOPBMsg()
        msg.sub_id = ParamIOSubId.SET
        msg.param_id = param
        msg.param_type = parameter_encoding(param)

        # Serialize the data to be sent
        if msg.param_type == MessageEncoding.STRING:
            msg.data = str(value).encode('utf-8')
        elif msg.param_type == MessageEncoding.FLOAT:
            msg.data = struct.pack('<f', float(value))
        elif msg.param_type == MessageEncoding.DOUBLE:
            msg.data = struct.pack('<d', float(value))
        elif msg.param_type == MessageEncoding.UINT8 or msg.param_type == MessageEncoding.BOOL:
            msg.data = struct.pack('<B', int(value))
        elif msg.param_type == MessageEncoding.UINT16:
            msg.data = struct.pack('<H', int(value))
        elif msg.param_type == MessageEncoding.UINT32:
            msg.data = struct.pack('<I', int(value))
        else:
            logger.error(f"Don't know how to serialize parameter type {msg.param_type}")
            return False

        # Push the request onto the wire
        rsp = self._com_pipe.write_and_wait(msg, 3.0)

        # Parse the result
        if not rsp:
            logger.error("No valid response from server")
            return False

        assert isinstance(rsp, AckNackPBMsg), f"Expected AckNackMessage but got {type(rsp)}"
        if not rsp.ack:
            logger.error(f"SET parameter {repr(param)} failed with status code: {repr(rsp.status_code)}")
            return False
        else:
            return True

    def load(self) -> bool:
        """
        Requests a complete reload from disk of all supported parameters
        Returns:
            True if the operation succeeds, False if not
        """
        msg = ParamIOPBMsg()
        msg.sub_id = ParamIOSubId.LOAD

        rsp = self._com_pipe.write_and_wait(msg, timeout=10.0)
        if not rsp:
            logger.error("No valid response from server")
            return False

        assert isinstance(rsp, AckNackPBMsg), f"Expected AckNackMessage but got {type(rsp)}"
        if not rsp.ack:
            logger.error(f"LOAD parameters failed with status code: {repr(rsp.status_code)}")
            return False
        else:
            return True

    def store(self) -> bool:
        """
        Requests a complete serialize to disk of all supported parameters
        Returns:
            True if the operation succeeds, False if not
        """
        msg = ParamIOPBMsg()
        msg.sub_id = ParamIOSubId.SYNC

        rsp = self._com_pipe.write_and_wait(msg, timeout=10.0)
        if not rsp:
            logger.error("No valid response from server")
            return False

        assert isinstance(rsp, AckNackPBMsg), f"Expected AckNackMessage but got {type(rsp)}"
        if not rsp.ack:
            logger.error(f"STORE failed with status code: {repr(rsp.status_code)}")
            return False
        else:
            return True
