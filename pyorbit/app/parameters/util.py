from typing import Optional, Union
from loguru import logger
from PyQt5 import QtWidgets, QtCore

from pyorbit.app.parameters.abstract import AbstractParameter
from pyorbit.serial.parameters import ParameterId

parameter_widget_lookup_cache = {}     # type: dict[str, AbstractParameter]
parameter_id_lookup_cache = {}         # type: dict[ParameterId, AbstractParameter]
parameter_listing_cache = []           # type: list[AbstractParameter]


def discover_parameters() -> None:
    """
    Discovers all the parameters in the application and caches them.
    Returns:
        None
    """
    import inspect
    import pyorbit.app.parameters.mapping as mapped_types

    # Get all classes that are a subclass of AbstractParameter and add them to parameter_listing_cache
    for name, obj in inspect.getmembers(mapped_types):
        if inspect.isclass(obj) and issubclass(obj, AbstractParameter):
            try:
                param_instance = obj()
                parameter_listing_cache.append(param_instance)
                parameter_id_lookup_cache[param_instance.parameter_id] = param_instance
            except TypeError:
                if obj.__name__ != "AbstractParameter":
                    logger.warning(f"Could not instantiate parameter {obj.__name__}")


def lookup_parameter(widget: QtCore.QObject) -> Optional[AbstractParameter]:
    """
    Takes a widget and returns the parameter that is associated with it
    Args:
        widget: Which widget to look up the parameter manager for.

    Returns:
        The parameter manager or None
    """
    # If we haven't cached the available parameters yet, do so now
    if not parameter_listing_cache:
        discover_parameters()

    # If the widget is not supported, return None
    supported_widget_types = (QtWidgets.QDoubleSpinBox, QtWidgets.QLineEdit, QtWidgets.QSpinBox)
    if not any(isinstance(widget, widget_type) for widget_type in supported_widget_types):
        logger.warning(f"Widget {widget} is not supported by the parameter manager")
        return None

    # If we've already cached this widget, return it
    if widget.objectName() in parameter_widget_lookup_cache.keys():
        return parameter_widget_lookup_cache[widget.objectName()]

    # Otherwise perform a more expensive lookup by comparing the widget name to the parameter object name
    for parameter in parameter_listing_cache:
        if parameter.object_name == widget.objectName():
            parameter_widget_lookup_cache[widget.objectName()] = parameter
            return parameter


def update_parameter(source_widget: QtCore.QObject, value: Union[float, int, str, bool]) -> None:
    """
    Updates the parameter associated with the source widget with the given value.
    Args:
        source_widget: The widget that triggered the update.
        value: The new value of the parameter.

    Returns:
        None
    """
    param = lookup_parameter(source_widget)
    if param:
        param.value = value
    else:
        logger.warning(f"Could not find parameter for widget {source_widget.objectName()}")