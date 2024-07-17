import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QOpenGLWidget
from PyQt5.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import pygame

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

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.5

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

        # Draw solid red cube
        glPushMatrix()
        glTranslatef(self.x_pos, self.y_pos, self.z_pos)
        glRotatef(self.x_rot, 1.0, 0.0, 0.0)
        glRotatef(self.y_rot, 0.0, 1.0, 0.0)
        glRotatef(self.z_rot, 0.0, 0.0, 1.0)
        glColor3f(1.0, 0.0, 0.0)
        glutSolidCube(1.0)
        glPopMatrix()
    
        # Draw wireframe black cube (outline)
        glPushMatrix()
        glTranslatef(self.x_pos, self.y_pos, self.z_pos)
        glRotatef(self.x_rot, 1.0, 0.0, 0.0)
        glRotatef(self.y_rot, 0.0, 1.0, 0.0)
        glRotatef(self.z_rot, 0.0, 0.0, 1.0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glLineWidth(3.0)
        glColor3f(0.0, 0.0, 0.0)
        glutSolidCube(1.0)
        glPopMatrix()

        # Ensure OpenGL state is reset to default after rendering
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)  # Reset polygon mode to fill
        glLineWidth(1.0)  # Reset line width

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

        # Initialize Pygame for joystick input
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_controller)
        self.timer.start(16)  # Update approximately every 16 milliseconds (about 60 FPS)

    def initUI(self):
        layout = QVBoxLayout(self)
        self.setLayout(layout)

        self.glWidget = GLWidget(self)
        layout.addWidget(self.glWidget)

        button = QPushButton('Connect to Drone', self)
        layout.addWidget(button)

        self.setWindowTitle('Collaborative Control Interface')
        self.setGeometry(50, 50, 800, 600)

    def update_controller(self):
        def apply_deadzone(value, threshold):
            return value if abs(value) > threshold else 0

        pygame.event.pump()
        
        # Deadzone threshold
        DEADZONE = 0.1
        
        # Get axis values for sticks and apply deadzone
        left_stick_x = apply_deadzone(self.controller.get_axis(0), DEADZONE)
        left_stick_y = apply_deadzone(self.controller.get_axis(1), DEADZONE)
        right_stick_x = apply_deadzone(self.controller.get_axis(3), DEADZONE)
        right_stick_y = apply_deadzone(self.controller.get_axis(4), DEADZONE)
        
        # Get bumper button states
        left_bumper = self.controller.get_button(4)  # Typically button index 4 for left bumper
        right_bumper = self.controller.get_button(5) # Typically button index 5 for right bumper
        
        # Move object on xy plane
        self.glWidget.x_pos += left_stick_x * 0.1
        self.glWidget.z_pos += left_stick_y * 0.1
        
        # Move object up and down on y axis with bumpers
        if left_bumper:
            self.glWidget.y_pos -= 0.1
        if right_bumper:
            self.glWidget.y_pos += 0.1
        
        # Rotate object with right stick
        self.glWidget.x_rot += right_stick_y *2.0
        self.glWidget.z_rot -= right_stick_x *2.0

def main():
    glutInit(sys.argv)  # Initialize GLUT before starting the QApplication
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
