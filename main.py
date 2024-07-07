import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QOpenGLWidget
from PyQt5.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class GLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(16)  # Update approximately every 16 milliseconds (about 60 FPS)

        self.x_rot = 0.0
        self.y_rot = 0.0
        self.z_rot = 0.0

    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glEnable(GL_DEPTH_TEST)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width() / self.height(), 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)

        # Draw grid
        glColor3f(0.68, 0.68, 0.68)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i, 0, -10)
            glVertex3f(i, 0, 10)
            glVertex3f(-10, 0, i)
            glVertex3f(10, 0, i)
        glEnd()

        # Draw axes
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(1, 0, 0)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1, 0)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1)
        glEnd()

        # Draw a colored box
        glPushMatrix()
        glTranslatef(0, 0, 0.5)
        glRotatef(self.x_rot, 1.0, 0.0, 0.0)
        glRotatef(self.y_rot, 0.0, 1.0, 0.0)
        glRotatef(self.z_rot, 0.0, 0.0, 1.0)
        glColor3f(1.0, 0.0, 0.0)
        glutSolidCube(1.0)
        glPopMatrix()

        self.x_rot += 1.0
        self.y_rot += 0.5
        self.z_rot += 0.25

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        self.setLayout(layout)

        self.glWidget = GLWidget(self)
        layout.addWidget(self.glWidget)

        button = QPushButton('Connect to Drone', self)
        layout.addWidget(button)

        self.setWindowTitle('Collaborative Control Interface')
        self.setGeometry(50, 50, 800, 600)

def main():
    glutInit(sys.argv)  # Initialize GLUT before starting the QApplication
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
