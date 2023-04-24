from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class PyOrbitGUI:

    def __init__(self):
        self._app = QApplication([])
        self._window = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(QPushButton('Top'))
        layout.addWidget(QPushButton('Bottom'),)
        self._window.setLayout(layout)
        self._window.show()

    def run(self):
        self._app.exec_()


if __name__ == '__main__':
    gui = PyOrbitGUI()
    gui.run()
