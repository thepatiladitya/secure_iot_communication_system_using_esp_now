"""
Module      : Vision Controller (PC Software)
Role        : Computer vision–based navigation & system orchestration
Functions   :
  - Detects LED-based sensor nodes using OpenCV
  - Commands car controller via TCP
  - Triggers ESP-NOW sensor sampling via SD ESP32
  - Initiates secure serial transfer via Base ESP32
  - Logs sensor data into CSV files
Platform    : PC / Laptop (Linux / Windows)
"""
import cv2
import numpy as np
import socket
import time
import csv
import datetime
import threading
import serial
import os
import sys

# ======== CONFIGURATION ========
STREAM_URL = "http://10.17.34.245/stream"  # ESP32-CAM
CAR_IP = "10.17.34.42"
CAR_PORT = 12345
SD_IP = "10.17.34.230"
SD_PORT = 5000
BASE_SERIAL_PORT = "/dev/ttyUSB0"  # Change to "COM3" on Windows
BAUD_RATE = 115200
CALIB_FILE = "node_calibration.txt"

# ======== ROTATION CONFIGURATION ========
ROTATION_ANGLE_DEG = 7  # ← CHANGE THIS TO MATCH YOUR ESP32 CODE
MAX_ROTATIONS = int(round(360 / ROTATION_ANGLE_DEG))  # ~52 for 7°
ROTATE_DELAY_SEC = 0.35

def load_calibration():
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, 'r') as f:
                K_val = float(f.read().strip())
                print(f"💾 Loaded calibration: K = {K_val:.2f}")
                return K_val
        except Exception as e:
            print(f"⚠️ Failed to load calibration file: {e}. Using default K=3000.")
    return 3000.0

def save_calibration(K_val):
    try:
        with open(CALIB_FILE, 'w') as f:
            f.write(f"{K_val:.6f}")
        print(f"💾 Calibration saved: K = {K_val:.2f}")
    except Exception as e:
        print(f"❌ Failed to save calibration: {e}")

K = load_calibration()
min_width = 5
min_distance_cm = 10
SMOOTH_ALPHA = 0.30

# Alignment tolerance — ONLY green center tolerance used for acceptance
IMAGE_WIDTH = 640
IMAGE_CENTER_X = IMAGE_WIDTH // 2
CENTER_TOLERANCE_NORMAL = 20    # Must be within this to count as "centered"

# ======== GLOBAL STATE ========
latest_distance = None
smoothed_distance = None
mission_state = "IDLE"
selected_node = 1  # 1 = Green, 2 = Blue
scanning = False
scan_target = 1
rotation_sent = False
last_rotate_time = 0
rotation_count = 0
auto_proceed_pending = False
auto_proceed_time = 0
stabilization_delay = 4.0  # ← NOW 4 SECONDS

kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# ======== CAMERA SETUP ========
def connect_camera(url, retries=10, wait=2):
    host = url.replace("http://", "").split("/")[0]
    for attempt in range(1, retries + 1):
        print(f"🔄 Attempt {attempt}/{retries}: Checking if {host} is reachable...")
        try:
            sock = socket.create_connection((host, 80), timeout=3)
            sock.close()
        except:
            print(f"❌ {host} not reachable. Retrying in {wait}s...")
            time.sleep(wait)
            continue

        cap = cv2.VideoCapture(url)
        time.sleep(1.5)
        if not cap.isOpened():
            cap.release()
            time.sleep(wait)
            continue

        valid = False
        for _ in range(5):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                valid = True
                break
            time.sleep(0.8)
        if not valid:
            cap.release()
            time.sleep(wait)
            continue

        print("✅ Camera connected!\n")
        return cap
    return None

# ======== CAR COMMUNICATION ========
def send_car_command_async(command):
    def _send():
        global mission_state
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((CAR_IP, CAR_PORT))
            sock.sendall(f"{command}\n".encode('utf-8'))
            
            if command in ["APPROACH_NODE", "ROTATE"]:
                response = sock.recv(128).decode('utf-8', errors='ignore').strip()
                if response != "OK":
                    print(f"⚠️ Car warning: {response}")
                    mission_state = "IDLE"
                    sock.close()
                    return

                print(f"📤 Command '{command}' accepted by car.")
                mission_state = "MOVING"

                while mission_state in ["MOVING", "SAMPLING"]:
                    try:
                        data = sock.recv(128)
                        if not data:
                            break
                        msg = data.decode('utf-8', errors='ignore').strip()
                        if msg.startswith("STATUS:"):
                            status = msg[len("STATUS:"):]
                            print(f"🔔 Car status: {status}")
                            if status == "REACHED_NODE":
                                mission_state = "SAMPLING"
                                start_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                trigger_sampling_with_time(start_time)
                            elif status == "BACK_AT_BASE":
                                mission_state = "READY_TO_TRANSFER"
                                break
                    except socket.timeout:
                        continue
                sock.close()
            else:
                print(f"📤 Speed command '{command}' sent.")
                sock.close()

        except Exception as e:
            print(f"❌ Car communication failed: {e}")
            if command in ["APPROACH_NODE", "ROTATE"]:
                mission_state = "IDLE"
    threading.Thread(target=_send, daemon=True).start()

# ======== TRIGGER SAMPLING ========
def trigger_sampling_with_time(timestamp):
    global selected_node
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect((SD_IP, SD_PORT))
            command = f"r,{selected_node},{timestamp}\n"
            s.sendall(command.encode('utf-8'))
            resp = s.recv(256).decode('utf-8', errors='ignore').strip()
            print(f"📡 SD ESP32 response: {resp}")
    except Exception as e:
        print(f"❌ Failed to trigger sampling: {e}")

# ======== SECURE TRANSFER ========
def secure_transfer():
    global selected_node
    try:
        ser = serial.Serial(BASE_SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(1.5)
        ser.write(b'\x01')
        print("📤 Secure transfer triggered. Waiting for sensor data...")

        records = []
        start_time = time.time()
        while time.time() - start_time < 12:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "END":
                break
            if line and not any(line.startswith(prefix) for prefix in ["📡", "❌", "🔔", "📤", "✅", "⚠️", "📞", "🔌", "🕒", "🟢"]):
                if selected_node == 1:
                    parts = line.split(',', 2)
                    if len(parts) >= 3:
                        try:
                            float(parts[0]); float(parts[1])
                            records.append((parts[2].strip(), parts[0].strip(), parts[1].strip()))
                        except ValueError:
                            continue
                else:
                    parts = line.split(',', 1)
                    if len(parts) >= 2:
                        try:
                            float(parts[0])
                            records.append((parts[1].strip(), parts[0].strip()))
                        except ValueError:
                            continue
        ser.close()

        if records:
            now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            if selected_node == 1:
                filename = f"dht_readings_{now_str}.csv"
                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        "Data Sensed Timing",
                        "Sensor Data",
                        "",
                        "Data Received Time at Base Station"
                    ])
                    writer.writerow([
                        "",
                        "Temperature (°C)",
                        "Humidity (%)",
                        ""
                    ])
                    for esp_time, temp, hum in records:
                        pc_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        writer.writerow([esp_time, temp, hum, pc_ts])
                print(f"✅ DHT11 data saved to: {filename}")
            else:
                filename = f"mq6_readings_{now_str}.csv"
                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        "Data Sensed Timing",
                        "Gas Concentration (ppm)",
                        "Data Received Time at Base Station"
                    ])
                    for esp_time, gas_val in records:
                        pc_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        writer.writerow([esp_time, gas_val, pc_ts])
                print(f"✅ MQ-6 data saved to: {filename}")
        else:
            print("📭 No valid sensor data received.")
    except Exception as e:
        print(f"❌ Serial transfer failed: {e}")

# ======== LED DETECTION (Brightness + Ring Color) ========
def detect_node(frame, target_node):
    if frame.size == 0:
        return False, None, None, None, np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    _, mask_bright = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    mask_bright = cv2.morphologyEx(mask_bright, cv2.MORPH_CLOSE, kernel_close)
    mask_bright = cv2.morphologyEx(mask_bright, cv2.MORPH_OPEN, kernel_open)

    contours, _ = cv2.findContours(mask_bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if not (min_width <= w <= 180 and min_width <= h <= 180):
            continue
        area = cv2.contourArea(cnt)
        if area < 50:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < 0.3:
            continue

        center_x = x + w // 2
        center_y = y + h // 2
        radius = max(w, h) // 2

        ring_mask = np.zeros_like(gray)
        outer_r = int(radius * 0.85)
        inner_r = int(radius * 0.25)
        if outer_r <= inner_r:
            continue

        cv2.circle(ring_mask, (center_x, center_y), outer_r, 255, -1)
        cv2.circle(ring_mask, (center_x, center_y), inner_r, 0, -1)

        ring_pixels = hsv[ring_mask == 255]
        if len(ring_pixels) < 15:
            continue

        median_H = np.median(ring_pixels[:, 0])
        median_S = np.median(ring_pixels[:, 1])

        if median_S < 20:
            continue

        detected_color = "unknown"
        if 40 <= median_H <= 90:
            detected_color = "green"
        elif 100 <= median_H <= 140:  # Blue range in OpenCV HSV
            detected_color = "blue"

        expected_color = "green" if target_node == 1 else "blue"
        if detected_color == expected_color:
            distance = max(K / max(w, h), min_distance_cm)
            mask_display = np.zeros_like(gray)
            cv2.drawContours(mask_display, [cnt], -1, 255, -1)
            return True, distance, cnt, center_x, mask_display

    return False, None, None, None, np.zeros_like(gray)

# ======== MANUAL SPEED CONTROL ========
def listen_for_manual_commands():
    while True:
        try:
            user_input = input().strip()
            if not user_input:
                continue

            if user_input.lower() == 'q':
                print("⌨️  Quit command from terminal.")
                os._exit(0)
            if user_input.startswith('s') and user_input[1:].isdigit():
                speed = int(user_input[1:])
                if 0 <= speed <= 255:
                    send_car_command_async(f"SPEED_FWD {speed}")
                else:
                    print("⚠️ Speed must be between 0 and 255.")
            elif user_input.startswith('r') and user_input[1:].isdigit():
                speed = int(user_input[1:])
                if 0 <= speed <= 255:
                    send_car_command_async(f"SPEED_ROT {speed}")
                else:
                    print("⚠️ Speed must be between 0 and 255.")
            else:
                print("❓ Unknown command.")
                print("   Use: s<0-255> → set forward speed")
                print("        r<0-255> → set rotation speed")
                print("        q → quit")
        except EOFError:
            break
        except Exception as e:
            print(f"⌨️ Input error: {e}")

# ======== MAIN LOOP ========
def main():
    global latest_distance, smoothed_distance, mission_state, selected_node
    global scanning, scan_target, rotation_sent, last_rotate_time, rotation_count
    global auto_proceed_pending, auto_proceed_time, K

    cap = connect_camera(STREAM_URL)
    if cap is None:
        print("🛑 Camera failed. Exiting.")
        return

    cmd_thread = threading.Thread(target=listen_for_manual_commands, daemon=True)
    cmd_thread.start()

    print("🎮 Controls (in OpenCV window):")
    print("  '1' → Scan for Node 1 (Green LED - DHT11)")
    print("  '2' → Scan for Node 2 (Blue LED - MQ-6)")
    print("  'e' → Calibrate distance")
    print("  'g' → Start mission to centered node")
    print("  's' → Secure transfer after return")
    print("  'q' → Quit\n")
    print("💬 In TERMINAL: s80 (forward speed), r60 (rotation speed), q (quit)\n")
    print(f"🔄 Scanning with {MAX_ROTATIONS} steps (~{ROTATION_ANGLE_DEG}° each)")
    print(f"⏱️  Stabilization delay: {stabilization_delay} seconds (must be centered on green line)")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None or frame.size == 0:
                print("⚠️ Lost frame. Reconnecting...")
                cap.release()
                time.sleep(2)
                cap = connect_camera(STREAM_URL)
                if cap is None:
                    break
                continue

            frame = cv2.resize(frame, (IMAGE_WIDTH, 480))
            current_time = time.time()

            if auto_proceed_pending:
                if current_time - auto_proceed_time >= stabilization_delay:
                    print(f"✅ Starting mission to Node {selected_node} at {latest_distance:.1f} cm...")
                    send_car_command_async("APPROACH_NODE")
                    auto_proceed_pending = False

            # === SCANNING LOGIC ===
            if scanning:
                if not rotation_sent:
                    send_car_command_async("ROTATE")
                    rotation_sent = True
                    last_rotate_time = current_time
                    rotation_count += 1
                    print(f"🔄 Rotation {rotation_count}/{MAX_ROTATIONS} sent. Waiting {ROTATE_DELAY_SEC}s...")

                elif current_time - last_rotate_time >= ROTATE_DELAY_SEC:
                    time.sleep(0.1)
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        rotation_sent = False
                        continue

                    frame = cv2.resize(frame, (IMAGE_WIDTH, 480))
                    detected, distance, contour, center_x, mask = detect_node(frame, scan_target)

                    # 🔑 CRITICAL CHANGE: Only accept if within GREEN (normal) tolerance
                    if detected and abs(center_x - IMAGE_CENTER_X) <= CENTER_TOLERANCE_NORMAL:
                        print(f"🎯 Node {scan_target} perfectly centered ({center_x}px)! Stabilizing for {stabilization_delay}s...")
                        smoothed_distance = distance
                        latest_distance = distance
                        selected_node = scan_target
                        scanning = False
                        rotation_sent = False
                        rotation_count = 0
                        mission_state = "IDLE"
                        auto_proceed_pending = True
                        auto_proceed_time = current_time
                    else:
                        if detected:
                            offset = abs(center_x - IMAGE_CENTER_X)
                            print(f"   → Node seen but NOT centered (offset={offset}px > {CENTER_TOLERANCE_NORMAL}px). Continuing scan.")
                        if rotation_count >= MAX_ROTATIONS:
                            print("❌ Full 360° scan: node not found within center tolerance.")
                            scanning = False
                            rotation_sent = False
                            rotation_count = 0
                            mission_state = "IDLE"
                        else:
                            rotation_sent = False

            # === NORMAL MODE ===
            if not scanning and not auto_proceed_pending:
                detected, distance, contour, center_x, mask = detect_node(frame, selected_node)
                if detected and abs(center_x - IMAGE_CENTER_X) <= CENTER_TOLERANCE_NORMAL:
                    if smoothed_distance is None:
                        smoothed_distance = distance
                    else:
                        smoothed_distance = SMOOTH_ALPHA * distance + (1 - SMOOTH_ALPHA) * smoothed_distance
                    latest_distance = smoothed_distance
                    x, y, w, h = cv2.boundingRect(contour)
                    bgr_color = (0, 255, 0) if selected_node == 1 else (255, 0, 0)  # Blue = (255,0,0) in BGR
                    cv2.rectangle(frame, (x, y), (x + w, y + h), bgr_color, 2)
                    cv2.line(frame, (center_x, y), (center_x, y + h), (0, 255, 0), 2)
                else:
                    latest_distance = None
                    smoothed_distance = None
            else:
                _, _, _, _, mask = detect_node(frame, scan_target if scanning else selected_node)

            # === DISPLAY ===
            color_name = "Green LED (Node 1 - DHT11)" if selected_node == 1 else "Blue LED (Node 2 - MQ-6)"
            distance_text = f"Distance: {latest_distance:.1f} cm" if latest_distance is not None else "Distance: N/A"
            state_text = f"State: {mission_state} | Target: {color_name} | K={K:.0f}"
            if scanning:
                state_text += " [SCANNING...]"
            if auto_proceed_pending:
                wait_time = stabilization_delay - (current_time - auto_proceed_time)
                state_text += f" [Auto-start in {max(0, wait_time):.1f}s]"

            bgr_color = (0, 255, 0) if selected_node == 1 else (255, 0, 0)
            cv2.putText(frame, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, bgr_color, 2)
            cv2.putText(frame, state_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # ====== ALIGNMENT LINES: ONLY 3 LINES (green center + 2 orange) ======
            cv2.line(frame, (IMAGE_CENTER_X, 0), (IMAGE_CENTER_X, 480), (0, 255, 0), 1)  # Green center
            cv2.line(frame, (IMAGE_CENTER_X - CENTER_TOLERANCE_NORMAL, 0),
                     (IMAGE_CENTER_X - CENTER_TOLERANCE_NORMAL, 480), (0, 150, 255), 1)  # Orange left
            cv2.line(frame, (IMAGE_CENTER_X + CENTER_TOLERANCE_NORMAL, 0),
                     (IMAGE_CENTER_X + CENTER_TOLERANCE_NORMAL, 480), (0, 150, 255), 1)  # Orange right

            cv2.imshow("Navigation", frame)
            cv2.imshow("Mask", mask)

            # === KEY HANDLING ===
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('1'):
                if mission_state != "IDLE" or auto_proceed_pending:
                    print("⏳ Finish current action first.")
                else:
                    selected_node = 1
                    if latest_distance is not None and not scanning:
                        print("✅ Node 1 already centered.")
                    else:
                        print("🔍 Starting scan for Node 1 (Green LED)...")
                        scanning = True
                        scan_target = 1
                        rotation_sent = False
                        rotation_count = 0
                        auto_proceed_pending = False
            elif key == ord('2'):
                if mission_state != "IDLE" or auto_proceed_pending:
                    print("⏳ Finish current action first.")
                else:
                    selected_node = 2
                    if latest_distance is not None and not scanning:
                        print("✅ Node 2 already centered.")
                    else:
                        print("🔍 Starting scan for Node 2 (Blue LED)...")
                        scanning = True
                        scan_target = 2
                        rotation_sent = False
                        rotation_count = 0
                        auto_proceed_pending = False
            elif key == ord('e'):
                if mission_state != "IDLE" or scanning or auto_proceed_pending:
                    print("⏳ System busy. Wait until idle.")
                else:
                    detected, _, contour, _, _ = detect_node(frame, selected_node)
                    if not detected:
                        print("❌ Node not visible. Center it first.")
                    else:
                        x, y, w, h = cv2.boundingRect(contour)
                        width = max(w, h)
                        try:
                            true_dist = float(input("📏 Enter TRUE distance to node (cm): "))
                            if true_dist <= 0:
                                print("⚠️ Distance must be positive.")
                                continue
                            old_K = K
                            K = true_dist * width
                            save_calibration(K)
                            print(f"✅ K updated: {old_K:.1f} → {K:.1f} (width={width}px at {true_dist}cm)")
                            smoothed_distance = None
                            latest_distance = None
                        except ValueError:
                            print("⚠️ Invalid number.")
                        except Exception as ex:
                            print(f"⚠️ Calibration error: {ex}")
            elif key == ord('g'):
                if mission_state != "IDLE":
                    print("⏳ Mission in progress.")
                elif auto_proceed_pending:
                    print("🛑 Auto-proceed scheduled.")
                elif latest_distance is None:
                    print("⚠️ No centered node. Scan first.")
                else:
                    _, _, _, center_x, _ = detect_node(frame, selected_node)
                    if center_x is not None and abs(center_x - IMAGE_CENTER_X) <= CENTER_TOLERANCE_NORMAL:
                        print(f"🚀 Starting mission to {color_name} at {latest_distance:.1f} cm...")
                        send_car_command_async("APPROACH_NODE")
                    else:
                        print("⚠️ Node moved. Re-center.")
            elif key == ord('s'):
                if mission_state == "READY_TO_TRANSFER":
                    print("🔐 Secure transfer...")
                    secure_transfer()
                    mission_state = "IDLE"
                else:
                    print("❌ Press 's' only after car returns.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\n👋 Program terminated.")

if __name__ == "__main__":
    main()
