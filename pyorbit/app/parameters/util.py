from typing import Optional, Union, List, Dict
from loguru import logger
from PyQt5 import QtWidgets, QtCore

from pyorbit.app.parameters.abstract import AbstractParameter
from pyorbit.serial.parameters import ParameterId, MessageEncoding

parameter_widget_lookup_cache: Dict[str, AbstractParameter] = {}
parameter_id_lookup_cache: Dict[ParameterId, AbstractParameter] = {}
parameter_listing_cache: List[AbstractParameter] = []


def discover_parameters() -> None:
    """
    Discovers all the parameters in the application and caches an instance of each them.
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


def valid_parameter_ids() -> List[ParameterId]:
    """
    Returns:
        A list of all valid parameter ids.
    """
    if not parameter_listing_cache:
        discover_parameters()

    return [parameter.parameter_id for parameter in parameter_listing_cache]


def parameter_encoding(pid: ParameterId) -> Optional[MessageEncoding]:
    """
    Returns the encoding of the given parameter id.
    Args:
        pid: The parameter id to lookup.

    Returns:
        The encoding of the parameter or None if it doesn't exist.
    """
    if not parameter_listing_cache:
        discover_parameters()

    if pid in parameter_id_lookup_cache.keys():
        return parameter_id_lookup_cache[pid].value_encoding
    else:
        logger.error(f"Could not find parameter {pid}")
        return None


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
    if param := lookup_parameter(source_widget):
        param.value = value
    else:
        logger.warning(f"Could not find parameter for widget {source_widget.objectName()}")
