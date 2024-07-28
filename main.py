import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QOpenGLWidget
from PyQt5.QtCore import QTimer, Qt
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import pygame

class SceneObject:
    def __init__(self, x, y, z, x_rot, y_rot, z_rot, color, size, length=0.5, transparency=1.0):
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z
        self.x_rot = x_rot
        self.y_rot = y_rot
        self.z_rot = z_rot
        self.color = color
        self.size = size
        self.length = length
        self.transparency = transparency

        self.x_vel = 0.0  # Adding velocity attributes
        self.y_vel = 0.0
        self.z_vel = 0.0

    def draw(self):
        # Draw solid rectangle
        glPushMatrix()
        glTranslatef(self.x_pos, self.y_pos, self.z_pos)
        glRotatef(self.x_rot, 1.0, 0.0, 0.0)
        glRotatef(self.y_rot, 0.0, 1.0, 0.0)
        glRotatef(self.z_rot, 0.0, 0.0, 1.0)
        glScalef(self.length, self.size, self.size)
        glColor4f(*self.color, self.transparency)
        glutSolidCube(1.0)
        glPopMatrix()

        # Draw wireframe rectangle
        glPushMatrix()
        glTranslatef(self.x_pos, self.y_pos, self.z_pos)
        glRotatef(self.x_rot, 1.0, 0.0, 0.0)
        glRotatef(self.y_rot, 0.0, 1.0, 0.0)
        glRotatef(self.z_rot, 0.0, 0.0, 1.0)
        glScalef(self.length, self.size, self.size)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glLineWidth(3.0)
        glColor3f(0.0, 0.0, 0.0)
        glutSolidCube(1.0)
        glPopMatrix()

        # Ensure OpenGL state is reset to default after rendering
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)  # Reset polygon mode to fill
        glLineWidth(1.0)  # Reset line width

class GLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(16)  # Update approximately every 16 milliseconds (about 60 FPS)

        self.objects = []

        # Initialize some objects
        self.objects.append(SceneObject(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, (1.0, 0.0, 0.0), 0.5))
        self.objects.append(SceneObject(1.5, 0.0, 0.5, 0.0, 0.0, 0.0, (0.0, 0.0, 1.0), 0.5, transparency=0.5))
        self.objects.append(SceneObject(3.0, 0.0, 1.5, 0.0, 0.0, 0.0, (0.0, 1.0, 0.0), 0.25, length=3.0))  # Green rectangle

        # Camera parameters
        self.camera_distance = 10.0
        self.camera_angle_x = 30.0
        self.camera_angle_y = 45.0
        self.mouse_last_x = 0
        self.mouse_last_y = 0
        self.mouse_left_button_down = False

    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width() / self.height(), 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(
            self.camera_distance * np.sin(np.radians(self.camera_angle_y)) * np.cos(np.radians(self.camera_angle_x)),
            self.camera_distance * np.sin(np.radians(self.camera_angle_x)),
            self.camera_distance * np.cos(np.radians(self.camera_angle_y)) * np.cos(np.radians(self.camera_angle_x)),
            0.0, 0.0, 0.0,
            0.0, 1.0, 0.0
        )

        self.draw_grid()
        self.draw_axes()

        for obj in self.objects:
            obj.draw()
        
        # Draw dashed line between the first and second objects
        start = np.array([self.objects[0].x_pos, self.objects[0].y_pos, self.objects[0].z_pos])
        end = np.array([self.objects[1].x_pos, self.objects[1].y_pos, self.objects[1].z_pos])
        self.draw_dashed_line(start, end, (0.855, 0.647, 0.125))  # Yellow color


        # Draw a red arrow in the direction of the first object's movement
        arrow_start = np.array([self.objects[0].x_pos, self.objects[0].y_pos, self.objects[0].z_pos])
        arrow_end = arrow_start + np.array([self.objects[0].x_vel, self.objects[0].y_vel, self.objects[0].z_vel]) * 10  # Scale the velocity for visibility
        self.draw_arrow(arrow_start, arrow_end, (1.0, 0.0, 0.0))  # Red color arrow




    def draw_grid(self):
        glColor3f(0.68, 0.68, 0.68)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i, 0, -10)
            glVertex3f(i, 0, 10)
            glVertex3f(-10, 0, i)
            glVertex3f(10, 0, i)
        glEnd()

    def draw_axes(self):
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

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_left_button_down = True
            self.mouse_last_x = event.x()
            self.mouse_last_y = event.y()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_left_button_down = False

    def mouseMoveEvent(self, event):
        if self.mouse_left_button_down:
            dx = event.x() - self.mouse_last_x
            dy = event.y() - self.mouse_last_y
            self.camera_angle_y += dx * 0.5
            self.camera_angle_x -= dy * 0.5
            self.camera_angle_x = max(-90, min(90, self.camera_angle_x))
            self.mouse_last_x = event.x()
            self.mouse_last_y = event.y()
            self.update()
    
    def draw_dashed_line(self, start, end, color, dash_length=0.1):
        glColor3f(*color)
        glLineWidth(4.0)  # Set line width to 3.0 for thicker lines
        glBegin(GL_LINES)   
        length = np.linalg.norm(np.array(end) - np.array(start))
        num_dashes = int(length / dash_length)
        for i in range(num_dashes):
            t1 = i / num_dashes
            t2 = (i + 0.5) / num_dashes
            point1 = start * (1 - t1) + np.array(end) * t1
            point2 = start * (1 - t2) + np.array(end) * t2
            glVertex3fv(point1)
            glVertex3fv(point2)
        glEnd()
        glLineWidth(1.0)  # Set line width to 3.0 for thicker lines

    def draw_arrow(self, start, end, color, arrow_head_length=0.2, arrow_head_width=0.1):
        glColor3f(*color)
        glLineWidth(3.0)  # Set line width for the arrow

        # Draw the arrow shaft
        glBegin(GL_LINES)
        glVertex3fv(start)
        glVertex3fv(end)
        glEnd()

        # Calculate the direction of the arrow
        direction = np.array(end) - np.array(start)
        direction = direction / np.linalg.norm(direction)

        # Calculate points for the arrow head
        ortho_direction = np.array([-direction[1], direction[0], 0])  # Perpendicular in 2D
        arrow_left = np.array(end) - arrow_head_length * direction + arrow_head_width * ortho_direction
        arrow_right = np.array(end) - arrow_head_length * direction - arrow_head_width * ortho_direction

        # Draw the arrow head
        glBegin(GL_TRIANGLES)
        glVertex3fv(end)
        glVertex3fv(arrow_left)
        glVertex3fv(arrow_right)
        glEnd()

        glLineWidth(1.0)  # Reset line width


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
        
        DEADZONE = 0.1
        
        left_stick_x = apply_deadzone(self.controller.get_axis(0), DEADZONE)
        left_stick_y = apply_deadzone(self.controller.get_axis(1), DEADZONE)
        right_stick_x = apply_deadzone(self.controller.get_axis(3), DEADZONE)
        right_stick_y = apply_deadzone(self.controller.get_axis(4), DEADZONE)
        
        left_bumper = self.controller.get_button(4)
        right_bumper = self.controller.get_button(5)
        
        # Update velocities
        self.glWidget.objects[0].x_vel = left_stick_x * 0.1
        self.glWidget.objects[0].z_vel = left_stick_y * 0.1
        self.glWidget.objects[0].y_vel = (right_bumper - left_bumper) * 0.1
        
        # Update positions based on velocities
        self.glWidget.objects[0].x_pos += self.glWidget.objects[0].x_vel
        self.glWidget.objects[0].z_pos += self.glWidget.objects[0].z_vel
        self.glWidget.objects[0].y_pos += self.glWidget.objects[0].y_vel
        
        self.glWidget.objects[0].x_rot += right_stick_y * 2.0
        self.glWidget.objects[0].z_rot -= right_stick_x * 2.0

def main():
    glutInit(sys.argv)  # Initialize GLUT before starting the QApplication
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
