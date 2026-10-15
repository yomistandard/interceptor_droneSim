"""Target controller: moves the target Mavic 2 Pro in a smooth circle,
and disarms (falls) if intercepted.
"""
import math
from controller import Supervisor

RADIUS = 8.0
HEIGHT = 5.0
SPEED = 1.0  # m/s along the circle

robot = Supervisor()
dt_ms = int(robot.getBasicTimeStep())

target_node = robot.getSelf()
translation_field = target_node.getField("translation")
rotation_field = target_node.getField("rotation")

# Find the interceptor node to monitor distance
interceptor_node = robot.getFromDef("interceptor")  # Ensure your wbt has DEF interceptor or search by name

intercepted = False

while robot.step(dt_ms) != -1:
    t = robot.getTime()

    # Check if intercepted by measuring distance to interceptor
    if interceptor_node is not None and not intercepted:
        target_pos = translation_field.getSFVec3f()
        interceptor_pos = interceptor_node.getPosition()
        dist = math.dist(target_pos, interceptor_pos)

        if dist < 1.5:
            intercepted = True
            print("TARGET: Intercepted! Disarming and falling...")

    if intercepted:
        # Disarmed: Do not update circular position, let physics/gravity take over
        # (by stopping translation updates, gravity pulls the drone down naturally)
        continue

    # Normal circular flight path
    ang = SPEED / RADIUS * t
    x = RADIUS * math.cos(ang)
    y = RADIUS * math.sin(ang)
    z = HEIGHT

    translation_field.setSFVec3f([x, y, z])
    yaw = ang + math.pi / 2.0
    rotation_field.setSFRotation([0, 0, 1, yaw])