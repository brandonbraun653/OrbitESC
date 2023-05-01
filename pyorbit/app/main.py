from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore, QtGui
from pyorbit.app.ui.pyorbit import Ui_MainWindow
from pyorbit.app.connections import SerialConnection
from loguru import logger


class PyOrbitGUI(QMainWindow, Ui_MainWindow):

    appTearDownSignal = QtCore.pyqtSignal()

    def __init__(self):
        self._app = QApplication([])
        super().__init__()
        self.setupUi(self)
        self.show()

        # Run the serial connection manager in the background.
        self._serial_connection = SerialConnection()
        self._serial_connection.start()
        self.appTearDownSignal.connect(self._serial_connection.tear_down)
        self.serialTargetComboBox.targetChosenSignal.connect(self._serial_connection.selected_target)
        self.connectDisconnectButton.stateChangeSignal.connect(self._serial_connection.connect_button_clicked)

        # Connect the console output visualizer
        self._serial_connection.onOpenSignal.connect(self.consoleOutput.attach_serial_client)
        self._serial_connection.onOpenSignal.connect(self.graphWidget.attach_serial_client)

        self._serial_connection.onCloseSignal.connect(self.consoleOutput.detach_serial_client)
        self._serial_connection.onCloseSignal.connect(self.graphWidget.detach_serial_client)

    def run(self) -> int:
        return self._app.exec()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        # Publish the appTearDownSignal to all connected slots.
        logger.debug("Emitting kill signal")
        self.appTearDownSignal.emit()

        # Wait for all worker threads to finish
        self._serial_connection.wait()
        logger.debug("Closing PyOrbit GUI")
        self._app.quit()


if __name__ == '__main__':
    import sys
    gui = PyOrbitGUI()
    sys.exit(gui.run())
