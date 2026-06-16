from pymavlink import mavutil
import time
import math

drone = mavutil.mavlink_connection('udp:127.0.0.1:14551')
drone.wait_heartbeat()
print("Connected!")

def goto(drone, lat, lon, alt):
    drone.mav.set_position_target_global_int_send(
        0, drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)

def get_pos(drone):
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg:
        return msg.lat/1e7, msg.lon/1e7
    return None, None

def distance(lat1, lon1, lat2, lon2):
    R = 6371000
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))

drone.mav.set_mode_send(drone.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
time.sleep(1)
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
print("Arming...")
time.sleep(3)
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 15)
print("Takeoff! 15m...")
time.sleep(8)

d_lat, d_lon = get_pos(drone)
target_lat = d_lat + 0.0002
target_lon = d_lon
target_alt = 15.0

move_lat = 0.000003
move_lon = 0.000003

print("\n Target chase shuru!")
print("-" * 50)

step = 0
while True:
    step += 1
    target_lat += move_lat
    target_lon += move_lon
    d_lat, d_lon = get_pos(drone)
    if d_lat is None:
        continue
    dist = distance(d_lat, d_lon, target_lat, target_lon)
    print(f"Step {step:03d} | Dist: {dist:.1f}m | Target:({target_lat:.6f},{target_lon:.6f})")
    if dist < 3.0:
        print("\n TARGET INTERCEPTED!")
        break
    goto(drone, target_lat, target_lon, target_alt)
    time.sleep(0.3)

print("Landing...")
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(5)
print("Done!")
