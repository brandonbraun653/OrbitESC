import math
import time
from typing import List
from pyorbit.serial.messages import SystemDataPBMsg, SystemTickPBMsg
from pyorbit.tests.fixtures import *
from pyorbit.nanopb.system_data_pb2 import *
from pyorbit.serial.client import OrbitClient
from pyorbit.utility.time import period_to_hz


@pytest.mark.usefixtures("serial_client")
class TestStaticStreamingData:
    """ Validates non-configurable periodic data messages that are expected to be received from the OrbitESC """

    def test_system_voltages(self, serial_client: OrbitClient) -> None:
        """ Validates that system voltages are being reported periodically """

        packets: List[SystemDataPBMsg] = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemDataPBMsg) and (msg.data_id == SystemDataId.ADC_SYSTEM_VOLTAGES),
            qty=5,
            timeout=5.0)
        assert len(packets) == 5

        messages = [msg.extract_payload() for msg in packets]

        # Validate the periodicity of the messages
        for idx in range(1, len(messages)):
            hz = period_to_hz(packets[idx].timestamp, packets[idx - 1].timestamp)
            assert isinstance(messages[idx], ADCSystemVoltagesPayload)
            assert 0.0 < messages[idx].v_dc_link < 18.0
            assert 1.64 < messages[idx].v_isense < 1.66
            assert 3.25 < messages[idx].v_mcu < 3.35
            assert 0.0 < messages[idx].v_temp < 3.3
            assert 4 < hz < 6

    def test_system_tick(self, serial_client: OrbitClient) -> None:
        """ Validates that system tick messages are being reported periodically """

        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemTickPBMsg),
            qty=5,
            timeout=5.0)
        assert len(packets) == 5

        # Validate the periodicity of the messages
        for idx in range(1, len(packets)):
            assert isinstance(packets[idx], SystemTickPBMsg)
            hz = period_to_hz(packets[idx].tick, packets[idx - 1].tick, unit=1e-3)
            assert 9 < hz < 11

    def test_motor_phase_currents(self, serial_client: OrbitClient) -> None:
        """ Validates that motor phase current data is being reported periodically """

        # Enable the streaming of phase current data
        serial_client.stream_phase_currents(True)
        time.sleep(0.5)

        # Listen for the data
        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemDataPBMsg) and (msg.data_id == SystemDataId.ADC_PHASE_CURRENTS),
            qty=15,
            timeout=5.0)

        # Disable streaming
        serial_client.stream_phase_currents(False)

        messages = [msg.extract_payload() for msg in packets]

        # Validate the periodicity and data of the messages
        assert len(packets) == 15
        avg_hz = 0.0
        for idx in range(1, len(messages)):
            avg_hz += period_to_hz(packets[idx].timestamp, packets[idx - 1].timestamp)

            # Ensure the message converted over to the expected type
            assert isinstance(messages[idx], ADCPhaseCurrentsPayload)

            # The motor drive isn't enabled, so the reported currents should be roughly zero
            assert math.isclose(messages[idx].ia, 0.0, rel_tol=0.1)
            assert math.isclose(messages[idx].ib, 0.0, rel_tol=0.1)
            assert math.isclose(messages[idx].ic, 0.0, rel_tol=0.1)

        # The reported frequency should be roughly 30Hz
        avg_hz /= float(len(messages) - 1)
        assert math.isclose(avg_hz, 30.0, rel_tol=0.05)


