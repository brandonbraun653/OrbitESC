import math
from abc import ABCMeta, abstractmethod
from typing import Union
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication
from loguru import logger
from pyorbit.serial.client import SerialClient
from pyorbit.serial.parameters import ParameterId


class AbstractParameter(metaclass=ABCMeta):
    """ Abstract class for representing state of UI parameters """

    def __init__(self):
        self._new_value = None
        self._esc_value = None

    @property
    @abstractmethod
    def parameter_id(self) -> ParameterId:
        """ ParameterId of the parameter, used for communicating with the target node """
        pass

    @property
    @abstractmethod
    def object_name(self) -> str:
        """ The object name of the widget that displays the parameter value """
        pass

    @property
    @abstractmethod
    def widget_type(self) -> type:
        """ The type of the widget that displays the parameter value """
        pass

    @property
    def value(self) -> Union[float, int, str, bool]:
        """
        Returns:
            The current value of the parameter.
        """
        return self._new_value

    @value.setter
    def value(self, value: Union[float, int, str, bool]) -> None:
        """
        Sets the current value of the parameter.
        Args:
            value: The new value of the parameter.

        Returns:
            None
        """
        self._new_value = value

    @property
    def dirty(self) -> bool:
        """
        Returns:
            True if the parameter value has changed since the last refresh, False otherwise.
        """
        return self._new_value != self._esc_value

    def apply(self, serial: SerialClient) -> None:
        """
        Applies the parameter value to the target node.
        Args:
            serial: The SerialClient instance to use for communicating with the target node.

        Returns:
            None
        """
        if not self.dirty:
            logger.trace(f"Parameter {self.parameter_id} is not dirty, skipping apply")
            return

        # Apply the parameter, then read it back to verify that it was applied correctly
        logger.info(f"Applying parameter {self.parameter_id} with value {self.value}")
        if serial.parameter.set(self.parameter_id, self.value):
            programmed_value = serial.parameter.get(self.parameter_id)

            # Compare the programmed value to the value we tried to program
            if type(programmed_value) == float:
                is_equal = math.isclose(programmed_value, self.value, rel_tol=1e-5)
            else:
                is_equal = programmed_value == self.value

            if is_equal:
                self._esc_value = self.value
                logger.debug(f"Successfully applied parameter {self.parameter_id} with value {self.value}")

        # At this point, if the parameter is still dirty, we failed to apply it
        if self.dirty:
            logger.warning(f"Failed to apply parameter {self.parameter_id} with value {self.value}")

    def refresh(self, serial: SerialClient) -> None:
        """
        Refreshes the parameter value from the target node.
        Args:
            serial: The SerialClient instance to use for communicating with the target node.

        Returns:
            None
        """
        logger.trace(f"Refreshing parameter {self.parameter_id}")
        programmed_value = serial.parameter.get(self.parameter_id)
        if programmed_value is not None:
            # Update the caches to the new state, clearing the dirty flag
            self._esc_value = programmed_value
            self._new_value = programmed_value

            # Find the widget that displays the parameter value and update it
            window = QApplication.activeWindow()
            widget = window.findChild(self.widget_type, self.object_name)
            success = False
            if isinstance(widget, QtWidgets.QAbstractSpinBox):
                widget.setValue(programmed_value)
                success = True
            elif isinstance(widget, QtWidgets.QLineEdit):
                widget.setText(str(programmed_value))
                success = True
            else:
                logger.error(f"Unhandled widget type {widget} for parameter {self.parameter_id}")

            if success:
                logger.debug(f"Successfully refreshed parameter {self.parameter_id} with value {self.value}")
