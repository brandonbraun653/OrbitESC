from typing import Optional
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QCheckBox, QDial, QDoubleSpinBox, QLCDNumber, QSlider
from pyorbit.app.connections.serial import SerialConnectionManagerSingleton, get_serial_client
from pyorbit.serial.intf.system_control import ManualCurrentControlToggleMessage


class InnerLoopTabWidget(QtWidgets.QWidget):
    """ Custom event handler for the 'Inner Loop' tab in the GUI """

    def __init__(self):
        super().__init__()
        self._manual_override_check_box = None  # type: Optional[QCheckBox]
        self._engage_drive_check_box = None  # type: Optional[QCheckBox]
        self._theta_dial = None  # type: Optional[QDial]
        self._theta_lcd = None  # type: Optional[QLCDNumber]
        self._d_axis_kp_spin_box = None  # type: Optional[QDoubleSpinBox]
        self._d_axis_ki_spin_box = None  # type: Optional[QDoubleSpinBox]
        self._d_axis_ref_lcd = None  # type: Optional[QLCDNumber]
        self._d_axis_ref_slider = None  # type: Optional[QSlider]
        self._q_axis_kp_spin_box = None  # type: Optional[QDoubleSpinBox]
        self._q_axis_ki_spin_box = None  # type: Optional[QDoubleSpinBox]
        self._q_axis_ref_lcd = None  # type: Optional[QLCDNumber]
        self._q_axis_ref_slider = None  # type: Optional[QSlider]
        self._d_axis_output_voltage_lcd = None  # type: Optional[QLCDNumber]
        self._q_axis_output_voltage_lcd = None  # type: Optional[QLCDNumber]
        self._d_axis_measured_current_lcd = None  # type: Optional[QLCDNumber]
        self._q_axis_measured_current_lcd = None  # type: Optional[QLCDNumber]

    @property
    def manual_override(self) -> QCheckBox:
        """
        Lazily loads the manual override checkbox object
        Returns:
            Checkbox object, if available, else None
        """
        if not self._manual_override_check_box:
            self._manual_override_check_box = self.findChild(QCheckBox, "checkBox_ManualOverride")

        return self._manual_override_check_box

    @property
    def engage_drive(self) -> QCheckBox:
        """
        Lazily loads the engage drive checkbox object
        Returns:
            Checkbox object, if available, else None
        """
        if not self._engage_drive_check_box:
            self._engage_drive_check_box = self.findChild(QCheckBox, "checkBox_EngageDrive")

        return self._engage_drive_check_box

    @property
    def theta_dial(self) -> QDial:
        """
        Lazily loads the theta dial object
        Returns:
            Dial object, if available, else None
        """
        if not self._theta_dial:
            self._theta_dial = self.findChild(QDial, "dial_Theta")

        return self._theta_dial

    @property
    def theta_lcd(self) -> QLCDNumber:
        """
        Lazily loads the theta lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._theta_lcd:
            self._theta_lcd = self.findChild(QLCDNumber, "lcdNumber_RotorThetaValue")

        return self._theta_lcd

    @property
    def d_axis_kp_spin_box(self) -> QDoubleSpinBox:
        """
        Lazily loads the d axis kp spin box object
        Returns:
            Spin box object, if available, else None
        """
        if not self._d_axis_kp_spin_box:
            self._d_axis_kp_spin_box = self.findChild(QDoubleSpinBox, "doubleSpinBox_DAxisKp")

        return self._d_axis_kp_spin_box

    @property
    def d_axis_ki_spin_box(self) -> QDoubleSpinBox:
        """
        Lazily loads the d axis ki spin box object
        Returns:
            Spin box object, if available, else None
        """
        if not self._d_axis_ki_spin_box:
            self._d_axis_ki_spin_box = self.findChild(QDoubleSpinBox, "doubleSpinBox_DAxisKi")

        return self._d_axis_ki_spin_box

    @property
    def d_axis_ref_lcd(self) -> QLCDNumber:
        """
        Lazily loads the d axis reference lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._d_axis_ref_lcd:
            self._d_axis_ref_lcd = self.findChild(QLCDNumber, "lcdNumber_IdReference")

        return self._d_axis_ref_lcd

    @property
    def d_axis_ref_slider(self) -> QSlider:
        """
        Lazily loads the d axis reference slider object
        Returns:
            Slider object, if available, else None
        """
        if not self._d_axis_ref_slider:
            self._d_axis_ref_slider = self.findChild(QSlider, "slider_IdRef")

        return self._d_axis_ref_slider

    @property
    def q_axis_kp_spin_box(self) -> QDoubleSpinBox:
        """
        Lazily loads the q axis kp spin box object
        Returns:
            Spin box object, if available, else None
        """
        if not self._q_axis_kp_spin_box:
            self._q_axis_kp_spin_box = self.findChild(QDoubleSpinBox, "doubleSpinBox_QAxisKp")

        return self._q_axis_kp_spin_box

    @property
    def q_axis_ki_spin_box(self) -> QDoubleSpinBox:
        """
        Lazily loads the q axis ki spin box object
        Returns:
            Spin box object, if available, else None
        """
        if not self._q_axis_ki_spin_box:
            self._q_axis_ki_spin_box = self.findChild(QDoubleSpinBox, "doubleSpinBox_QAxisKi")

        return self._q_axis_ki_spin_box

    @property
    def q_axis_ref_lcd(self) -> QLCDNumber:
        """
        Lazily loads the q axis reference lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._q_axis_ref_lcd:
            self._q_axis_ref_lcd = self.findChild(QLCDNumber, "lcdNumber_IqReference")

        return self._q_axis_ref_lcd

    @property
    def q_axis_ref_slider(self) -> QSlider:
        """
        Lazily loads the q axis reference slider object
        Returns:
            Slider object, if available, else None
        """
        if not self._q_axis_ref_slider:
            self._q_axis_ref_slider = self.findChild(QSlider, "slider_IqRef")

        return self._q_axis_ref_slider

    @property
    def d_axis_output_voltage_lcd(self) -> QLCDNumber:
        """
        Lazily loads the d axis output voltage lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._d_axis_output_voltage_lcd:
            self._d_axis_output_voltage_lcd = self.findChild(QLCDNumber, "lcdNumber_VdCommand")

        return self._d_axis_output_voltage_lcd

    @property
    def q_axis_output_voltage_lcd(self) -> QLCDNumber:
        """
        Lazily loads the q axis output voltage lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._q_axis_output_voltage_lcd:
            self._q_axis_output_voltage_lcd = self.findChild(QLCDNumber, "lcdNumber_VqCommand")

        return self._q_axis_output_voltage_lcd

    @property
    def d_axis_measured_current_lcd(self) -> QLCDNumber:
        """
        Lazily loads the d axis measured current lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._d_axis_measured_current_lcd:
            self._d_axis_measured_current_lcd = self.findChild(QLCDNumber, "lcdNumber_IdMeasured")

        return self._d_axis_measured_current_lcd

    @property
    def q_axis_measured_current_lcd(self) -> QLCDNumber:
        """
        Lazily loads the q axis measured current lcd object
        Returns:
            LCD object, if available, else None
        """
        if not self._q_axis_measured_current_lcd:
            self._q_axis_measured_current_lcd = self.findChild(QLCDNumber, "lcdNumber_IqMeasured")

        return self._q_axis_measured_current_lcd

    @QtCore.pyqtSlot()
    def on_serial_connect(self) -> None:
        self.reset_widgets_to_inactive_state()

    @QtCore.pyqtSlot()
    def on_serial_disconnect(self) -> None:
        self.reset_widgets_to_inactive_state()

    @QtCore.pyqtSlot(int)
    def on_theta_dial_value_changed(self, value: int) -> None:
        """
        Slot method for when the theta dial value changes in real time
        Args:
            value: New value of the theta dial

        Returns:
            None
        """
        self.theta_lcd.display(value)

    @QtCore.pyqtSlot()
    def on_theta_dial_value_applied(self) -> None:
        """
        Slot method for when the theta dial value is "selected" via releasing the mouse button
        Returns:
            None
        """
        # TODO: Need to ship a message to the target node to update the rotor angle
        pass

    @QtCore.pyqtSlot(int)
    def on_d_axis_ref_slider_value_changed(self, value: int) -> None:
        """
        Slot method for when the d axis reference slider value changes in real time
        Args:
            value: New value of the d axis reference slider

        Returns:
            None
        """
        self.d_axis_ref_lcd.display(float(value) / 1000.0)

    @QtCore.pyqtSlot()
    def on_d_axis_ref_slider_value_applied(self) -> None:
        """
        Slot method for when the d axis reference slider value is "selected" via releasing the mouse button
        Returns:
            None
        """
        pass

    @QtCore.pyqtSlot(int)
    def on_q_axis_ref_slider_value_changed(self, value: int) -> None:
        """
        Slot method for when the q axis reference slider value changes in real time
        Args:
            value: New value of the q axis reference slider

        Returns:
            None
        """
        self.q_axis_ref_lcd.display(float(value) / 1000.0)

    @QtCore.pyqtSlot()
    def on_q_axis_ref_slider_value_applied(self) -> None:
        """
        Slot method for when the q axis reference slider value is "selected" via releasing the mouse button
        Returns:
            None
        """
        pass

    @QtCore.pyqtSlot(int)
    def on_manual_override_state_changed(self, state: Qt.CheckState) -> None:
        """
        Slot method for when the manual override checkbox is clicked
        Args:
            state: The new state of the checkbox

        Returns:
            None
        """
        # TODO: Send message to target node to enable/disable manual override
        if state == Qt.CheckState.Checked:
            pass
        elif state == Qt.CheckState.Unchecked:
            pass

    @QtCore.pyqtSlot(int)
    def on_engage_drive_state_changed(self, state: Qt.CheckState) -> None:
        """
        Slot method for when the engage drive checkbox is clicked
        Args:
            state: The new state of the checkbox

        Returns:
            None
        """
        # TODO: Send message to target node to engage/disengage drive
        # if state == Qt.CheckState.Checked:
        #     pass
        # elif state == Qt.CheckState.Unchecked:
        #     pass
        get_serial_client().com_pipe.write(ManualCurrentControlToggleMessage().serialize())

    def reset_widgets_to_inactive_state(self) -> None:
        # Reset the manual override checkbox
        self.manual_override.setCheckState(Qt.CheckState.Unchecked)
        self.manual_override.stateChanged.connect(self.on_manual_override_state_changed)

        # Reset the engage drive checkbox
        self.engage_drive.setCheckState(Qt.CheckState.Unchecked)
        self.engage_drive.stateChanged.connect(self.on_engage_drive_state_changed)

        # Initialize the theta dial
        self.theta_dial.setValue(0)
        self.theta_dial.setRange(0, 359)
        self.theta_dial.valueChanged.connect(self.on_theta_dial_value_changed)
        self.theta_dial.sliderReleased.connect(self.on_theta_dial_value_applied)

        # Initialize the theta LCD
        self.theta_lcd.display(self.theta_dial.value())

        # Initialize the d axis reference slider
        self.d_axis_ref_slider.setValue(0)
        self.d_axis_ref_slider.valueChanged.connect(self.on_d_axis_ref_slider_value_changed)
        self.d_axis_ref_slider.sliderReleased.connect(self.on_d_axis_ref_slider_value_applied)

        # Initialize the d axis lcd
        self.d_axis_ref_lcd.setSmallDecimalPoint(True)
        self.d_axis_ref_lcd.display(self.d_axis_ref_slider.value())

        # Initialize the q axis reference slider
        self.q_axis_ref_slider.setValue(0)
        self.q_axis_ref_slider.valueChanged.connect(self.on_q_axis_ref_slider_value_changed)
        self.q_axis_ref_slider.sliderReleased.connect(self.on_q_axis_ref_slider_value_applied)

        # Initialize the q axis lcd
        self.q_axis_ref_lcd.setSmallDecimalPoint(True)
        self.q_axis_ref_lcd.display(self.q_axis_ref_slider.value())

        # Initialize the d axis kp spin box
        self.d_axis_kp_spin_box.setValue(0.0)
        self.d_axis_kp_spin_box.setSingleStep(0.1)
        self.d_axis_kp_spin_box.setDecimals(3)
        self.d_axis_kp_spin_box.setRange(0.0, 100.0)

        # Initialize the d axis ki spin box
        self.d_axis_ki_spin_box.setValue(0.0)
        self.d_axis_ki_spin_box.setSingleStep(0.1)
        self.d_axis_ki_spin_box.setDecimals(3)
        self.d_axis_ki_spin_box.setRange(0.0, 100.0)

        # Initialize the q axis kp spin box
        self.q_axis_kp_spin_box.setValue(0.0)
        self.q_axis_kp_spin_box.setSingleStep(0.1)
        self.q_axis_kp_spin_box.setDecimals(3)
        self.q_axis_kp_spin_box.setRange(0.0, 100.0)

        # Initialize the q axis ki spin box
        self.q_axis_ki_spin_box.setValue(0.0)
        self.q_axis_ki_spin_box.setSingleStep(0.1)
        self.q_axis_ki_spin_box.setDecimals(3)
        self.q_axis_ki_spin_box.setRange(0.0, 100.0)

        # Initialize the d axis output voltage lcd
        self.d_axis_output_voltage_lcd.setSmallDecimalPoint(True)
        self.d_axis_output_voltage_lcd.display(0.0)

        # Initialize the q axis output voltage lcd
        self.q_axis_output_voltage_lcd.setSmallDecimalPoint(True)
        self.q_axis_output_voltage_lcd.display(0.0)

        # Initialize the d axis measured current lcd
        self.d_axis_measured_current_lcd.setSmallDecimalPoint(True)
        self.d_axis_measured_current_lcd.display(0.0)

        # Initialize the q axis measured current lcd
        self.q_axis_measured_current_lcd.setSmallDecimalPoint(True)
        self.q_axis_measured_current_lcd.display(0.0)
