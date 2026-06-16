from pymavlink import mavutil
import time

drone = mavutil.mavlink_connection("udp:127.0.0.1:14551")
drone.wait_heartbeat()

drone.mav.set_mode_send(
    drone.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4
)
print("Guided mode set")
time.sleep(1)

drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("Arming...")
time.sleep(3)

drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10
)
time.sleep(8)

drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("Landing...")
time.sleep(5)
