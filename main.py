import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QGridLayout
import pyqtgraph.opengl as gl

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        # Create layout
        layout = QGridLayout()
        self.setLayout(layout)

        # Create left panel with buttons
        for i in range(5):
            button = QPushButton(f'Button {i+1}')
            layout.addWidget(button, i, 0)

        # Create bottom panel with fields
        for i in range(5):
            field = QLineEdit(f'Field {i+1}')
            layout.addWidget(field, 5, i)

        # Create 3D space with grid floor
        self.view = gl.GLViewWidget(self)
        layout.addWidget(self.view, 0, 1, 6, 5)  # Span the 3D view across the grid

        # Add a grid floor
        grid = gl.GLGridItem()
        self.view.addItem(grid)

        self.setGeometry(50, 50, 800, 600)
        self.setWindowTitle('PyQt5 Window')

def main():
    app = QApplication(sys.argv)

    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
