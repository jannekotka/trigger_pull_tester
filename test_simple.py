import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel

def main():
    print("Before QApplication")
    app = QApplication(sys.argv)
    print("After QApplication, before pyqtgraph")

    import pyqtgraph as pg
    print("After pyqtgraph import")

    label = QLabel("Hello PyQt6!", window)
    label.move(150, 80)

    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
