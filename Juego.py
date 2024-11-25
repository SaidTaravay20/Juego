import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import random

# Initialize Pygame and OpenGL
pygame.init()
display = (800, 600)
pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL)
gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
glTranslatef(0.0, -5.0, -30)

# Define a cube
cube_vertices = [
    [1, 1, -1],
    [1, -1, -1],
    [-1, -1, -1],
    [-1, 1, -1],
    [1, 1, 1],
    [1, -1, 1],
    [-1, -1, 1],
    [-1, 1, 1],
]

edges = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
]

# Game variables
cube_position = np.array([0.0, 0.0, 0.0])
cube_rotation = np.array([0.0, 0.0, 0.0])  # Rotation angles (X, Y, Z)
cube_scale = np.array([1.0, 1.0, 1.0])  # Scaling factors for X, Y, Z
score = 0

# Target shapes
target_shapes = [
    {"position": np.array([0.0, 0.0, -10.0]), "rotation": np.array([0.0, 0.0, 0.0]), "scale": np.array([1.0, 1.0, 1.0])},
    {"position": np.array([3.0, 2.0, -12.0]), "rotation": np.array([45.0, 45.0, 0.0]), "scale": np.array([1.5, 1.5, 1.5])},
    {"position": np.array([-2.0, -3.0, -15.0]), "rotation": np.array([90.0, 0.0, 90.0]), "scale": np.array([0.5, 0.5, 0.5])},
]
current_target = 0


def draw_cube(position, rotation, scale):
    glPushMatrix()
    glTranslatef(*position)
    glScalef(*scale)  # Apply scaling to the cube
    glRotatef(rotation[0], 1, 0, 0)
    glRotatef(rotation[1], 0, 1, 0)
    glRotatef(rotation[2], 0, 0, 1)
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(cube_vertices[vertex])
    glEnd()
    glPopMatrix()


def draw_target(target):
    glPushMatrix()
    glTranslatef(*target["position"])
    glScalef(*target["scale"])  # Apply scaling to the target
    glRotatef(target["rotation"][0], 1, 0, 0)
    glRotatef(target["rotation"][1], 0, 1, 0)
    glRotatef(target["rotation"][2], 0, 0, 1)
    glColor3f(0.0, 1.0, 0.0)  # Green for the target
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(cube_vertices[vertex])
    glEnd()
    glPopMatrix()


def check_alignment(cube_pos, cube_rot, cube_scale, target):
    """Check if the cube aligns with the target shape."""
    position_match = np.allclose(cube_pos, target["position"], atol=0.5)
    rotation_match = np.allclose(cube_rot, target["rotation"], atol=5.0)
    scale_match = np.allclose(cube_scale, target["scale"], atol=0.1)
    return position_match and rotation_match and scale_match


def rotate_vector(v, axis, angle):
    """Rotate vector v around axis by the given angle (in degrees)."""
    axis = axis / np.linalg.norm(axis)  # Normalize axis
    angle_rad = np.radians(angle)
    cos_theta = np.cos(angle_rad)
    sin_theta = np.sin(angle_rad)
    dot = np.dot(axis, v)
    cross = np.cross(axis, v)
    
    rotated_v = cos_theta * v + sin_theta * cross + (1 - cos_theta) * dot * axis
    return rotated_v


# Create a rotation matrix for rotation around an axis
def rotation_matrix(axis, angle):
    """Returns the rotation matrix for a given axis and angle (in degrees)."""
    axis = axis / np.linalg.norm(axis)  # Normalize axis
    angle_rad = np.radians(angle)
    cos_theta = np.cos(angle_rad)
    sin_theta = np.sin(angle_rad)
    ux, uy, uz = axis

    # Rotation matrix based on the axis-angle formula
    mat = np.array([
        [cos_theta + ux**2 * (1 - cos_theta), ux * uy * (1 - cos_theta) - uz * sin_theta, ux * uz * (1 - cos_theta) + uy * sin_theta],
        [uy * ux * (1 - cos_theta) + uz * sin_theta, cos_theta + uy**2 * (1 - cos_theta), uy * uz * (1 - cos_theta) - ux * sin_theta],
        [uz * ux * (1 - cos_theta) - uy * sin_theta, uz * uy * (1 - cos_theta) + ux * sin_theta, cos_theta + uz**2 * (1 - cos_theta)],
    ])
    return mat


# Apply the rotation matrix to the cube
def rotate_cube(cube_vertices, axis, angle):
    """Rotate the cube vertices using a rotation matrix."""
    rot_mat = rotation_matrix(axis, angle)
    rotated_vertices = []
    for vertex in cube_vertices:
        rotated_vertex = np.dot(rot_mat, vertex)
        rotated_vertices.append(rotated_vertex)
    return rotated_vertices


# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    # Move cube
    if keys[pygame.K_UP]:
        cube_position[1] += 0.1
    if keys[pygame.K_DOWN]:
        cube_position[1] -= 0.1
    if keys[pygame.K_LEFT]:
        cube_position[0] -= 0.1
    if keys[pygame.K_RIGHT]:
        cube_position[0] += 0.1
    if keys[pygame.K_w]:
        cube_position[2] += 0.1
    if keys[pygame.K_s]:
        cube_position[2] -= 0.1

    # Rotate cube using autovector (axis of rotation)
    if keys[pygame.K_a]:
        cube_rotation[1] -= 2.0  # Rotate around Y axis
    if keys[pygame.K_d]:
        cube_rotation[1] += 2.0  # Rotate around Y axis
    if keys[pygame.K_q]:
        cube_rotation[0] -= 2.0  # Rotate around X axis
    if keys[pygame.K_e]:
        cube_rotation[0] += 2.0  # Rotate around X axis
    if keys[pygame.K_z]:
        cube_rotation[2] -= 2.0  # Rotate around Z axis
    if keys[pygame.K_c]:
        cube_rotation[2] += 2.0  # Rotate around Z axis

    # Clear screen and render cube
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Rotate and draw cube
    rotated_cube_vertices = rotate_cube(cube_vertices, np.array([0, 1, 0]), cube_rotation[1])  # Rotate around Y axis
    draw_cube(cube_position, cube_rotation, cube_scale)

    # Check if the cube is aligned with the target shape
    if check_alignment(cube_position, cube_rotation, cube_scale, target_shapes[current_target]):
        score += 1
        print(f"Score: {score}")
        current_target = (current_target + 1) % len(target_shapes)

    # Draw target shape
    draw_target(target_shapes[current_target])

    pygame.display.flip()
    pygame.time.wait(10)

pygame.quit()
