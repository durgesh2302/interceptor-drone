from pymavlink import mavutil as mav
import cv2
import numpy as np
import math
import time
import threading

FRAME_W, FRAME_H = 640, 480
FOV = 90
MOVE_STEP = 0.0003

state = {
    'target_lat': 0.0, 'target_lon': 0.0,
    'drone_lat': 0.0,  'drone_lon': 0.0,
    'dist': 999.0, 'step': 0,
    'phase': 'INIT',
    'intercepted': False,
    'scan_angle': 0,
    'spd': 0.0,
    'target_moved': False,
}
key_move = {'lat': 0.0, 'lon': 0.0, 'pressed': False}

def gps_to_pixel(d_lat, d_lon, t_lat, t_lon):
    R = 6371000
    dlat = math.radians(t_lat - d_lat)
    dlon = math.radians(t_lon - d_lon)
    north = dlat * R
    east  = dlon * R * math.cos(math.radians(d_lat))
    dist  = math.sqrt(north**2 + east**2)
    if dist < 0.1: dist = 0.1
    scale = FRAME_W / (2 * math.tan(math.radians(FOV/2)))
    px = int(FRAME_W/2 + (east  / dist) * scale)
    py = int(FRAME_H/2 - (north / dist) * scale)
    return px, py, dist

def draw_frame():
    d_lat = state['drone_lat']
    d_lon = state['drone_lon']
    t_lat = state['target_lat']
    t_lon = state['target_lon']
    dist  = state['dist']
    phase = state['phase']
    cx, cy = FRAME_W//2, FRAME_H//2

    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    frame[:] = (15, 15, 15)

    if phase == 'SCANNING':
        state['scan_angle'] = (state['scan_angle'] + 4) % 360
        for r in range(30, 220, 45):
            cv2.circle(frame, (cx, cy), r, (0, 60, 0), 1)
        angle = state['scan_angle']
        ex = int(cx + 210 * math.cos(math.radians(angle)))
        ey = int(cy + 210 * math.sin(math.radians(angle)))
        cv2.line(frame, (cx, cy), (ex, ey), (0, 220, 0), 2)
        cv2.putText(frame, "SCANNING AREA...",
            (cx-90, FRAME_H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,220,0), 1)

    elif phase == 'RE-SCANNING':
        state['scan_angle'] = (state['scan_angle'] + 3) % 360
        cv2.circle(frame, (cx, cy), 60, (0, 180, 180), 1)
        cv2.circle(frame, (cx, cy), 120, (0, 100, 100), 1)
        sa = state['scan_angle']
        ex = int(cx + 120 * math.cos(math.radians(sa)))
        ey = int(cy + 120 * math.sin(math.radians(sa)))
        cv2.line(frame, (cx, cy), (ex, ey), (0, 200, 200), 1)
        cv2.putText(frame, "HOVERING - TARGET LOST",
            (cx-120, FRAME_H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,200), 1)

    # Crosshair
    cv2.line(frame, (cx-25, cy), (cx+25, cy), (0,255,0), 1)
    cv2.line(frame, (cx, cy-25), (cx, cy+25), (0,255,0), 1)
    cv2.circle(frame, (cx,cy), 35, (0,255,0), 1)

    if d_lat != 0 and phase not in ['SCANNING', 'INIT', 'ARMING', 'TAKEOFF']:
        px, py, _ = gps_to_pixel(d_lat, d_lon, t_lat, t_lon)
        in_frame = 0 < px < FRAME_W and 0 < py < FRAME_H

        if in_frame:
            box = max(10, int(160 / (dist/10 + 1)))
            if dist < 10:   color = (0, 255, 0)
            elif dist < 40: color = (0, 165, 255)
            else:           color = (0, 0, 255)

            cv2.rectangle(frame,
                (px-box, py-box), (px+box, py+box), color, 2)
            L = max(5, box//3)
            for dx, dy in [(-1,-1),(1,-1),(-1,1),(1,1)]:
                cv2.line(frame,
                    (px+dx*box, py+dy*box),
                    (px+dx*(box-L), py+dy*box), color, 2)
                cv2.line(frame,
                    (px+dx*box, py+dy*box),
                    (px+dx*box, py+dy*(box-L)), color, 2)
            cv2.circle(frame, (px,py), 4, color, -1)

            # Dotted line — center to target
            for i in range(1, 20):
                if i % 2 == 0:
                    ddx = int(cx + (px-cx) * i/20)
                    ddy = int(cy + (py-cy) * i/20)
                    cv2.circle(frame, (ddx, ddy), 2, (255,255,0), -1)

            cv2.putText(frame, f"TARGET {dist:.0f}m",
                (px-box, py-box-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            err_x = px - cx
            err_y = py - cy
            cv2.putText(frame, f"Err X:{err_x:+d} Y:{err_y:+d}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,0), 1)

        else:
            # Target out of frame — dotted line to edge + arrow
            # Clamp target position to frame edge
            dx = px - cx
            dy = py - cy
            angle = math.atan2(dy, dx)

            # Edge point calculate karo
            if abs(dx) > 0 and abs(dy) > 0:
                scale_x = (FRAME_W//2 - 20) / abs(dx) if dx != 0 else 999
                scale_y = (FRAME_H//2 - 20) / abs(dy) if dy != 0 else 999
                scale_e = min(scale_x, scale_y)
                edge_x = int(cx + dx * scale_e)
                edge_y = int(cy + dy * scale_e)
            else:
                edge_x = cx
                edge_y = cy

            edge_x = max(20, min(FRAME_W-20, edge_x))
            edge_y = max(20, min(FRAME_H-20, edge_y))

            # Dotted line center to edge
            length = int(math.sqrt((edge_x-cx)**2 + (edge_y-cy)**2))
            if length > 0:
                for i in range(0, length, 12):
                    t_val = i / length
                    ddx = int(cx + (edge_x-cx) * t_val)
                    ddy = int(cy + (edge_y-cy) * t_val)
                    cv2.circle(frame, (ddx, ddy), 2, (0,0,200), -1)

            # Arrow at edge
            cv2.arrowedLine(frame,
                (int(edge_x - 20*math.cos(angle)), int(edge_y - 20*math.sin(angle))),
                (edge_x, edge_y),
                (0, 0, 255), 2, tipLength=0.4)

            # Target out label
            cv2.putText(frame, f"TARGET OUT {dist:.0f}m",
                (cx-100, cy+60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)

            # Direction hint
            dirs = []
            if px > FRAME_W: dirs.append("EAST")
            if px < 0:       dirs.append("WEST")
            if py > FRAME_H: dirs.append("SOUTH")
            if py < 0:       dirs.append("NORTH")
            cv2.putText(frame, f"Direction: {' '.join(dirs)}",
                (cx-80, cy+90), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,200), 1)

    phase_colors = {
        'SCANNING':       (0,220,0),
        'RE-SCANNING':    (0,200,200),
        'LOCKED':         (0,255,255),
        'CHASING':        (0,80,255),
        'CLOSING':        (0,165,255),
        'INTERCEPT ZONE': (0,255,100),
        'INTERCEPTED':    (0,255,0),
        'LANDING':        (200,200,0),
    }
    pcol = phase_colors.get(phase, (180,180,180))
    cv2.putText(frame, phase, (10,25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, pcol, 2)
    cv2.putText(frame,
        f"Dist:{dist:.1f}m Spd:{state['spd']:.1f}m/s Step:{state['step']}",
        (10, FRAME_H-30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160,160,160), 1)
    cv2.putText(frame, "WASD: move target | Q: quit",
        (10, FRAME_H-12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100,100,100), 1)

    if state['intercepted']:
        overlay = frame.copy()
        cv2.rectangle(overlay,(cx-210,cy-45),(cx+210,cy+45),(0,60,0),-1)
        cv2.addWeighted(overlay,0.55,frame,0.45,0,frame)
        cv2.putText(frame, "TARGET INTERCEPTED!",
            (cx-170,cy+12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

    return frame

def mavlink_thread():
    drone = mav.mavlink_connection('udp:127.0.0.1:14551')
    drone.wait_heartbeat()
    print("[DRONE] Connected!")

    def goto(lat, lon, alt):
        drone.mav.set_position_target_global_int_send(
            0, drone.target_system, drone.target_component,
            mav.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat*1e7), int(lon*1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0)

    def get_pos():
        m = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if m: return m.lat/1e7, m.lon/1e7
        return None, None

    def distance(la1, lo1, la2, lo2):
        R = 6371000
        a = math.sin(math.radians(la2-la1)/2)**2 + \
            math.cos(math.radians(la1))*math.cos(math.radians(la2))* \
            math.sin(math.radians(lo2-lo1)/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    def mp_msg(text, sev=mav.mavlink.MAV_SEVERITY_INFO):
        drone.mav.statustext_send(sev, text.encode()[:50])

    for param, val in [(b'WPNAV_SPEED\x00\x00\x00\x00\x00', 1000.0),
                       (b'WPNAV_ACCEL\x00\x00\x00\x00\x00', 500.0)]:
        drone.mav.param_set_send(drone.target_system, drone.target_component,
            param, val, mav.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)

    drone.mav.set_mode_send(drone.target_system,
        mav.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    time.sleep(1)
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mav.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,1,0,0,0,0,0,0)
    state['phase'] = 'ARMING'
    print("[PHASE] ARMING")
    mp_msg("INTERCEPTOR ARMED")
    time.sleep(4)

    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mav.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,15)
    state['phase'] = 'TAKEOFF'
    print("[PHASE] TAKEOFF")
    mp_msg("TAKEOFF - INITIATING SCAN")
    time.sleep(10)

    state['phase'] = 'SCANNING'
    print("[PHASE] SCANNING")
    mp_msg("SCANNING AREA FOR TARGETS")
    time.sleep(5)

    d_lat, d_lon = get_pos()
    t_lat = d_lat + 0.0045
    t_lon = d_lon + 0.0020
    state['target_lat'] = t_lat
    state['target_lon'] = t_lon

    d0 = distance(d_lat, d_lon, t_lat, t_lon)
    state['phase'] = 'LOCKED'
    print(f"[PHASE] TARGET LOCKED — {d0:.0f}m")
    mp_msg(f"TARGET LOCKED {d0:.0f}m", mav.mavlink.MAV_SEVERITY_WARNING)
    mp_msg("ENGAGING INTERCEPT MODE", mav.mavlink.MAV_SEVERITY_WARNING)
    time.sleep(2)

    last_msg_time = time.time()
    out_count = 0
    hover_lat = d_lat
    hover_lon = d_lon
    step = 0

    while True:
        step += 1
        state['step'] = step
        now = time.time()

        # Key se target move
        if key_move['pressed']:
            t_lat += key_move['lat']
            t_lon += key_move['lon']
            key_move['lat'] = 0.0
            key_move['lon'] = 0.0
            key_move['pressed'] = False
            state['target_lat'] = t_lat
            state['target_lon'] = t_lon
            state['target_moved'] = True
            print(f"[TARGET MOVED] ({t_lat:.6f},{t_lon:.6f})")

        d_lat, d_lon = get_pos()
        if d_lat is None:
            continue
        state['drone_lat'] = d_lat
        state['drone_lon'] = d_lon

        d = distance(d_lat, d_lon, t_lat, t_lon)
        state['dist'] = d

        spd_m = drone.recv_match(type='VFR_HUD', blocking=False)
        if spd_m: state['spd'] = spd_m.groundspeed

        # Frame check
        px, py, _ = gps_to_pixel(d_lat, d_lon, t_lat, t_lon)
        in_frame = 0 < px < FRAME_W and 0 < py < FRAME_H

        # Out count sirf tab badhao jab target manually move hua ho
        if not in_frame and state['target_moved']:
            out_count += 1
        elif in_frame:
            out_count = 0
            hover_lat = d_lat
            hover_lon = d_lon
            # RE-SCAN se wapas aao
            if state['phase'] == 'RE-SCANNING':
                state['phase'] = 'CHASING'
                state['target_moved'] = False
                print(f"[PHASE] TARGET REACQUIRED — {d:.0f}m")
                mp_msg(f"TARGET REACQUIRED {d:.0f}m", mav.mavlink.MAV_SEVERITY_WARNING)

        # RE-SCAN trigger
        if out_count > 10 and state['phase'] != 'RE-SCANNING':
            state['phase'] = 'RE-SCANNING'
            out_count = 0
            print(f"[PHASE] TARGET LOST — HOVERING at ({hover_lat:.6f},{hover_lon:.6f})")
            mp_msg("TARGET LOST - HOVERING", mav.mavlink.MAV_SEVERITY_WARNING)

        # Phase update
        if state['phase'] != 'RE-SCANNING':
            if d > 50:   state['phase'] = 'CHASING'
            elif d > 15: state['phase'] = 'CLOSING'
            else:        state['phase'] = 'INTERCEPT ZONE'

        print(f"[{step:04d}] {state['phase']:15s} | "
              f"Dist:{d:7.1f}m | Spd:{state['spd']:5.1f}m/s | "
              f"Frame:{'IN' if in_frame else f'OUT({out_count})'}")

        if now - last_msg_time >= 1.0:
            if state['phase'] == 'RE-SCANNING':
                mp_msg("HOVERING - TARGET LOST", mav.mavlink.MAV_SEVERITY_WARNING)
            elif d > 50:
                mp_msg(f"CHASING {d:.0f}m SPD:{state['spd']:.1f}m/s")
            elif d > 15:
                mp_msg(f"CLOSING {d:.0f}m SPD:{state['spd']:.1f}m/s",
                    mav.mavlink.MAV_SEVERITY_WARNING)
            else:
                mp_msg(f"ALMOST THERE {d:.1f}m",
                    mav.mavlink.MAV_SEVERITY_CRITICAL)
            last_msg_time = now

        if d < 5.0:
            state['intercepted'] = True
            state['phase'] = 'INTERCEPTED'
            mp_msg("TARGET INTERCEPTED!!!", mav.mavlink.MAV_SEVERITY_EMERGENCY)
            print("\n" + "="*50)
            print(f"   TARGET INTERCEPTED! Step:{step} Dist:{d:.2f}m")
            print("="*50)
            time.sleep(3)
            break

        if state['phase'] == 'RE-SCANNING':
            goto(hover_lat, hover_lon, 15)
        else:
            goto(t_lat, t_lon, 15)
        time.sleep(0.3)

    mp_msg("MISSION COMPLETE - LANDING")
    print("[PHASE] LANDING")
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mav.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,0,0)
    state['phase'] = 'LANDING'

t = threading.Thread(target=mavlink_thread, daemon=True)
t.start()

print("="*50)
print("  INTERCEPTOR CAMERA")
print("  Click camera window → WASD to move target")
print("="*50)

while True:
    frame = draw_frame()
    cv2.imshow("Interceptor Camera View", frame)
    key = cv2.waitKey(33) & 0xFF

    if key == ord('q'):
        break
    elif key == ord('w') or key == 82:
        key_move['lat'] += MOVE_STEP
        key_move['pressed'] = True
        print("[KEY] W → NORTH")
    elif key == ord('s') or key == 84:
        key_move['lat'] -= MOVE_STEP
        key_move['pressed'] = True
        print("[KEY] S → SOUTH")
    elif key == ord('a') or key == 81:
        key_move['lon'] -= MOVE_STEP
        key_move['pressed'] = True
        print("[KEY] A → WEST")
    elif key == ord('d') or key == 83:
        key_move['lon'] += MOVE_STEP
        key_move['pressed'] = True
        print("[KEY] D → EAST")

    if state['intercepted'] and not t.is_alive():
        time.sleep(2)
        break

cv2.destroyAllWindows()
print("Done!")
