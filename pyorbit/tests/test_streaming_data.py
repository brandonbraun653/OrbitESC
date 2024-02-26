import csv
import math
import time
import logging
from pathlib import Path
from typing import List
from pyorbit.serial.messages import SystemDataPBMsg, SystemTickPBMsg, SystemStatusPBMsg
from pyorbit.tests.fixtures import *
from pyorbit.nanopb.system_data_pb2 import *
from pyorbit.nanopb.motor_control_pb2 import *
from pyorbit.serial.client import OrbitClient
from pyorbit.utility.time import period_to_hz

LOGGER = logging.getLogger(__name__)


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

    def test_system_status(self, serial_client: OrbitClient) -> None:
        """ Validates that system status messages are being reported periodically """

        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemStatusPBMsg),
            qty=5,
            timeout=5.0)
        assert len(packets) == 5

        # Validate the periodicity of the messages
        for idx in range(1, len(packets)):
            hz = period_to_hz(packets[idx].tick, packets[idx - 1].tick, unit=1e-3)
            assert 4 < hz < 6

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

    def test_motor_phase_voltages(self, serial_client: OrbitClient) -> None:
        """ Validates that motor phase voltage data is being reported periodically """

        LOGGER.info("Acquiring phase voltage data sample")
        serial_client.stream_phase_voltages(True)
        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemDataPBMsg) and (msg.data_id == SystemDataId.ADC_PHASE_VOLTAGES),
            qty=15,
            timeout=5.0)
        serial_client.stream_phase_voltages(False)

        messages = [msg.extract_payload() for msg in packets]

        LOGGER.info("Validating phase voltage data and frequency")
        assert len(packets) == 15
        avg_hz = 0.0
        for idx in range(1, len(messages)):
            avg_hz += period_to_hz(packets[idx].timestamp, packets[idx - 1].timestamp)

            # Ensure the message converted over to the expected type
            assert isinstance(messages[idx], ADCPhaseVoltagesPayload)

            # The motor drive isn't enabled, so the reported currents should be roughly zero
            assert math.isclose(messages[idx].va, 0.0, rel_tol=0.1)
            assert math.isclose(messages[idx].vb, 0.0, rel_tol=0.1)
            assert math.isclose(messages[idx].vc, 0.0, rel_tol=0.1)

        # The reported frequency should be roughly 30Hz
        avg_hz /= float(len(messages) - 1)
        assert math.isclose(avg_hz, 30.0, rel_tol=0.05)

    def test_phase_currents_when_armed(self, serial_client: OrbitClient) -> None:
        """ Validates expected phase currents when motor is armed and not spinning """
        LOGGER.info("Command transition to ARMED state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ARMED)

        LOGGER.info("Acquiring phase current data sample")
        assert serial_client.stream_phase_currents(True)

        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemDataPBMsg) and (msg.data_id == SystemDataId.ADC_PHASE_CURRENTS),
            qty=100,
            timeout=15)

        assert serial_client.stream_phase_currents(False)

        LOGGER.info("Command an IDLE state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

        LOGGER.info("Validating phase currents are approximately zero for all samples")
        messages = [msg.extract_payload() for msg in packets]
        assert len(packets) == 100
        for msg in messages:
            assert isinstance(msg, ADCPhaseCurrentsPayload)
            assert math.isclose(msg.ia, 0.0, rel_tol=0.1)
            assert math.isclose(msg.ib, 0.0, rel_tol=0.1)
            assert math.isclose(msg.ic, 0.0, rel_tol=0.1)

    def test_phase_voltages_when_armed(self, serial_client: OrbitClient) -> None:
        """ Validates expected phase voltages when motor is armed and not spinning """
        LOGGER.info("Command transition to ARMED state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ARMED)

        LOGGER.info("Acquiring phase voltage data sample")
        assert serial_client.stream_phase_voltages(True)

        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemDataPBMsg) and (msg.data_id == SystemDataId.ADC_PHASE_VOLTAGES),
            qty=100,
            timeout=15)

        assert serial_client.stream_phase_voltages(False)

        LOGGER.info("Command an IDLE state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

        LOGGER.info("Validating phase voltages are approximately zero for all samples")
        messages = [msg.extract_payload() for msg in packets]
        assert len(packets) == 100
        for msg in messages:
            assert isinstance(msg, ADCPhaseVoltagesPayload)
            assert math.isclose(msg.va, 0.0, rel_tol=0.1)
            assert math.isclose(msg.vb, 0.0, rel_tol=0.1)
            assert math.isclose(msg.vc, 0.0, rel_tol=0.1)

    def test_current_control_monitor_stream(self, serial_client: OrbitClient) -> None:
        """ Validates that the current control monitor data is being reported """
        LOGGER.info("Command transition to ENGAGED state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ARMED)
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ENGAGED)

        LOGGER.info("Acquiring current control loop monitor data")
        exp_quantity = 5000
        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemDataPBMsg) and (msg.data_id == SystemDataId.CURRENT_CONTROL_MONITOR),
            qty=exp_quantity,
            timeout=8)

        LOGGER.info("Command an IDLE state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

        LOGGER.info("Exporting data")
        messages = [msg.extract_payload() for msg in packets]
        # assert len(packets) == exp_quantity
        assert all(isinstance(msg, CurrentControlMonitorPayload) for msg in messages)

        # Write all the data to a CSV file for post-processing
        output_file = Path(__file__).parent / "data_output" / "current_control_monitor.csv"
        with output_file.open("w") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "ia", "ib", "ic", "id_ref", "iq_ref", "id", "iq", "vd", "vq", "va",
                             "vb"])
            for idx in range(len(messages)):
                writer.writerow([packets[idx].timestamp,
                                 messages[idx].ia, messages[idx].ib, messages[idx].ic,
                                 messages[idx].id_ref, messages[idx].iq_ref, messages[idx].id, messages[idx].iq,
                                 messages[idx].vd, messages[idx].vq, messages[idx].va, messages[idx].vb])

