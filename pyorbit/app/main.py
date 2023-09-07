from __future__ import annotations

from enum import Enum
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore, QtGui
from loguru import logger
from pyorbit.app.parameters.updater import ParameterUpdater
from pyorbit.app.ui.pyorbit import Ui_MainWindow
from pyorbit.app.connections.serial import SerialConnectionManagerSingleton
from pyorbit.serial.client import SerialClient

# Load our settings storage
AppSettings = QtCore.QSettings("OrbitESC", "PyOrbit")


class PyOrbitGUI(QMainWindow, Ui_MainWindow):
    appTearDownSignal = QtCore.pyqtSignal()
    appLogMessageSignal = QtCore.pyqtSignal(str)

    def __init__(self):
        self._app = QApplication([])
        super().__init__()
        self.setupUi(self)
        self.show()

        # Attach our custom logging sink to the loguru logger
        logger.add(self._loguru_log_message, level="DEBUG")

        # Run the serial connection manager in the background.
        self._serial_conn_mgr = SerialConnectionManagerSingleton()
        self._serial_conn_mgr.start()

        # Start the background parameter updater
        self._param_updater = ParameterUpdater()
        self._updater_thread = QtCore.QThread()
        self._updater_thread.start()
        self._param_updater.moveToThread(self._updater_thread)

    @property
    def settings(self) -> QtCore.QSettings:
        """
        Returns:
            Returns the application settings object.
        """
        return self._settings

    @property
    def serial_client(self) -> SerialClient:
        """
        Returns:
            Returns the serial client object from the serial connection manager.
        """
        return self._serial_conn_mgr.serial_client

    @property
    def parameter_updater(self) -> ParameterUpdater:
        """
        Returns:
            Returns the parameter updater object.
        """
        return self._param_updater

    def run(self) -> int:
        """
        Custom run method for the PyOrbitGUI application
        Returns:
            The exit code of the application
        """
        # Perform the initial setup of the GUI that doesn't require a running event loop.
        self._init_widgets()
        self._attach_signals()

        # Schedule the post-init function to run after the event loop has started.
        QTimer.singleShot(500, self._post_initialization)
        return self._app.exec()

    def showEvent(self, a0: QtGui.QShowEvent) -> None:
        # Load the last known window geometry from settings.
        if AppSettings.contains(Settings.WINDOW_GEOMETRY):
            self.restoreGeometry(AppSettings.value(Settings.WINDOW_GEOMETRY))

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        # Publish the appTearDownSignal to all connected slots.
        logger.debug("Emitting kill signal")
        self.appTearDownSignal.emit()

        # Wait for all worker threads to finish
        self._serial_conn_mgr.wait()
        logger.debug("Closing PyOrbit GUI")
        AppSettings.setValue(Settings.WINDOW_GEOMETRY, self.saveGeometry())
        AppSettings.sync()
        self._app.quit()

    def _init_widgets(self) -> None:
        """
        Initializes all widgets in the application that require manual configuration.
        Returns:
            None
        """
        from pyorbit.app.parameters.util import discover_parameters

        # Initialize the console window
        self.consoleWindow.init_console_window()

        # Now with the widgets initialized, we can discover parameters
        discover_parameters()

    def _attach_signals(self) -> None:
        """
        Forms connections between various signals and slots in the application.
        Returns:
            None
        """
        # Notify widgets of the app tear down signal.
        self.appTearDownSignal.connect(self._serial_conn_mgr.tear_down)

        # Notify widgets when a serial connection state change event is requested
        self.serialAttachButton.requestNewConnectionStateSignal.connect(self._serial_conn_mgr.connect_button_clicked)

        # Notify widgets of online/offline status of the node
        self._serial_conn_mgr.escOnlineStatusSignal.connect(self.serialAttachButton.notify_connection_state_change)

        # Notify widgets when a serial connection is opened
        self._serial_conn_mgr.onOpenSignal.connect(self.phaseCurrentPlotter.serial_connect)
        self._serial_conn_mgr.onOpenSignal.connect(self.speedPositionPlotter.serial_connect)
        self._serial_conn_mgr.onOpenSignal.connect(self._param_updater.on_serial_connect)
        self._serial_conn_mgr.onOpenSignal.connect(self.tabCurrentControl.on_serial_connect)

        # Notify widgets when a serial connection is closed
        self._serial_conn_mgr.onCloseSignal.connect(self.phaseCurrentPlotter.serial_disconnect)
        self._serial_conn_mgr.onCloseSignal.connect(self.speedPositionPlotter.serial_disconnect)
        self._serial_conn_mgr.onCloseSignal.connect(self._param_updater.on_serial_disconnect)
        self._serial_conn_mgr.onCloseSignal.connect(self.tabCurrentControl.on_serial_disconnect)

        # Notify widgets of a log message
        self.appLogMessageSignal.connect(self.consoleWindow.add_app_console_message)

        # Notify widgets of a live data stream request
        self.liveDataCheckBox.stateChanged.connect(self.phaseCurrentPlotter.toggle_live_data_stream)
        self.liveDataCheckBox.stateChanged.connect(self.speedPositionPlotter.toggle_live_data_stream)

        # Notify widgets of an auto-scale request
        self.autoScaleCheckBox.stateChanged.connect(self.phaseCurrentPlotter.toggle_auto_scale)
        self.autoScaleCheckBox.stateChanged.connect(self.speedPositionPlotter.toggle_auto_scale)

        # Notify widgets of a request to clear the current plot
        self.clearPlotButton.clicked.connect(self.phaseCurrentPlotter.clear_plot)
        self.clearPlotButton.clicked.connect(self.speedPositionPlotter.clear_plot)

        # Notify widgets of a request to Apply the current parameter settings
        self.applyParameterChangesButton.clicked.connect(self.parameterTabs.parameter_apply_clicked)

        # Notify widgets of a request to Refresh parameter settings
        self.refreshParameterButton.clicked.connect(self.parameterTabs.parameter_refresh_clicked)

        # Notify widgets of a request to Save parameter settings to a file
        self.saveParameterToFileButton.clicked.connect(self.parameterTabs.parameter_save_clicked)

        # Notify widgets of a request to Load parameter settings from a file
        self.loadParameterFromFileButton.clicked.connect(self.parameterTabs.parameter_load_clicked)

    def _post_initialization(self) -> None:
        """
        Performs any post-initialization tasks that require the application to be fully initialized.
        Returns:
            None
        """
        # Enqueue all parameters to be read from the ESC
        pass

    def _loguru_log_message(self, message: str) -> None:
        """
        Custom sink that receives a loguru log message and emits it to the appLogMessageSignal.
        Args:
            message: Message generated by Loguru

        Returns:
            None
        """
        self.appLogMessageSignal.emit(message)


class Settings(str, Enum):
    """ All available settings for PyOrbit """

    SERIAL_TARGET = "serial_target"
    PLOT_LIVE_DATA = "plot_live_data"
    PLOT_AUTO_SCALE = "plot_auto_scale"
    WINDOW_GEOMETRY = "window_geometry"


if __name__ == '__main__':
    import sys

    gui = PyOrbitGUI()
    sys.exit(gui.run())
