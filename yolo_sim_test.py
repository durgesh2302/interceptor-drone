import cv2
import numpy as np
import math

# Virtual camera settings
FRAME_W = 640
FRAME_H = 480
FOV = 90  # degrees

def gps_to_pixel(drone_lat, drone_lon, target_lat, target_lon, drone_heading=0):
    """GPS coordinates ko camera frame mein pixel position mein convert karo"""
    R = 6371000
    dlat = math.radians(target_lat - drone_lat)
    dlon = math.radians(target_lon - drone_lon)
    
    # Relative position meters mein
    north = dlat * R
    east = dlon * R * math.cos(math.radians(drone_lat))
    
    # Drone heading ke according rotate karo
    heading_rad = math.radians(drone_heading)
    cam_x = east * math.cos(heading_rad) - north * math.sin(heading_rad)
    cam_y = north * math.cos(heading_rad) + east * math.sin(heading_rad)
    
    # Pixel mein convert karo
    scale = FRAME_W / (2 * math.tan(math.radians(FOV/2)))
    
    # Distance
    dist = math.sqrt(north**2 + east**2)
    if dist < 0.1:
        dist = 0.1
    
    px = int(FRAME_W/2 + (cam_x / dist) * scale)
    py = int(FRAME_H/2 - (cam_y / dist) * scale)
    
    return px, py, dist

def draw_frame(drone_lat, drone_lon, target_lat, target_lon, dist):
    """Camera frame draw karo"""
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    frame[:] = (30, 30, 30)  # dark background
    
    # Crosshair — frame center
    cx, cy = FRAME_W//2, FRAME_H//2
    cv2.line(frame, (cx-20, cy), (cx+20, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, cy-20), (cx, cy+20), (0, 255, 0), 1)
    cv2.circle(frame, (cx, cy), 30, (0, 255, 0), 1)
    
    # Target pixel position
    px, py, _ = gps_to_pixel(drone_lat, drone_lon, target_lat, target_lon)
    
    # Target visible hai?
    if 0 < px < FRAME_W and 0 < py < FRAME_H:
        # Bounding box size distance ke according
        box_size = max(10, int(200 / (dist + 1)))
        
        # YOLO jaisi bounding box
        cv2.rectangle(frame,
            (px - box_size, py - box_size),
            (px + box_size, py + box_size),
            (0, 0, 255), 2)
        
        # Label
        cv2.putText(frame, f"TARGET {dist:.0f}m",
            (px - box_size, py - box_size - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Center dot
        cv2.circle(frame, (px, py), 4, (0, 0, 255), -1)
        
        # Pixel error
        err_x = px - cx
        err_y = py - cy
        
        # Error line
        cv2.line(frame, (cx, cy), (px, py), (255, 255, 0), 1)
        
        cv2.putText(frame, f"Error X:{err_x} Y:{err_y}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Dist: {dist:.1f}m",
            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        detected = True
    else:
        cv2.putText(frame, "TARGET NOT IN FRAME",
            (FRAME_W//2 - 100, FRAME_H//2 + 50),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        err_x, err_y = 0, 0
        detected = False
    
    return frame, detected, err_x if detected else None, err_y if detected else None

# Test — drone aur target positions
drone_lat  = -35.3633
drone_lon  =  149.1652
target_lat = -35.3610
target_lon =  149.1680

print("Virtual camera test shuru...")
print("Press Q to quit, Arrow keys to move target")

while True:
    _, _, dist = gps_to_pixel(drone_lat, drone_lon, target_lat, target_lon)
    frame, detected, ex, ey = draw_frame(drone_lat, drone_lon, target_lat, target_lon, dist)
    
    status = f"DETECTED err({ex},{ey})" if detected else "NOT DETECTED"
    print(f"\rDist:{dist:.1f}m | {status}    ", end="")
    
    cv2.imshow("Interceptor Virtual Camera", frame)
    
    key = cv2.waitKey(100) & 0xFF
    if key == ord('q'):
        break
    elif key == 82:  # up arrow
        target_lat += 0.0001
    elif key == 84:  # down arrow
        target_lat -= 0.0001
    elif key == 81:  # left arrow
        target_lon -= 0.0001
    elif key == 83:  # right arrow
        target_lon += 0.0001

cv2.destroyAllWindows()
print("\nDone!")
