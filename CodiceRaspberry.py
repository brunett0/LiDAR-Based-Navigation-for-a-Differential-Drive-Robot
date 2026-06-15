import numpy as np
import socket
import time
import RPi.GPIO as GPIO
from pyrplidar import PyRPlidar

# --- CONFIGURAZIONE ---
MATLAB_IP = "172.20.10.4"
PORT_LIDAR = 5007
PORT_CONTROL = 5005
GIRI_DA_ACCUMULARE = 5
K_RIGHT = 1.08

# --- SETUP MOTORI ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
pins = [20, 21, 26, 27, 22, 23, 24, 25]; pwm_pins = [0, 13, 1, 12]
for p in pins + pwm_pins: GPIO.setup(p, GPIO.OUT)
pwms = [GPIO.PWM(p, 100) for p in pwm_pins]
for p in pwms: p.start(0)

def set_motors(v_sx, v_dx):
    v_dx_corr = v_dx * K_RIGHT
    p_sx = min(abs(v_sx) * 100, 100); p_dx = min(abs(v_dx_corr) * 100, 100)
    GPIO.output(20, 0 if v_sx >= 0 else 1); GPIO.output(21, 1 if v_sx >= 0 else 0)
    GPIO.output(26, 1 if v_sx >= 0 else 0); GPIO.output(27, 0 if v_sx >= 0 else 1)
    GPIO.output(22, 0 if v_dx >= 0 else 1); GPIO.output(23, 1 if v_dx >= 0 else 0)
    GPIO.output(24, 1 if v_dx >= 0 else 0); GPIO.output(25, 0 if v_dx >= 0 else 1)
    for i, val in enumerate([p_sx, p_sx, p_dx, p_dx]):
        try: pwms[i].ChangeDutyCycle(val)
        except: pass

sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.bind(("0.0.0.0", PORT_CONTROL)); sock_in.setblocking(0)
sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def run():
    lidar = PyRPlidar()
    try:
        lidar.connect(port='/dev/ttyUSB0', baudrate=460800, timeout=3)
        lidar.set_motor_pwm(600); time.sleep(2)
        scan_gen = lidar.start_scan()
        print("ROM3 V35 - BUGFIX START_FLAG PRONTO.")

        collecting = False
        wait_for_sync = False
        rotation_count = 0
        target_giri = GIRI_DA_ACCUMULARE
        point_batch = []
        motor_stop_time = 0

        punti_totali_giro = 0
        punti_scartati_vicini = 0

        for scan in scan_gen():
            t_now = time.time()
            if motor_stop_time > 0 and t_now >= motor_stop_time:
                set_motors(0, 0); motor_stop_time = 0

            try:
                data, _ = sock_in.recvfrom(1024)
                msg = data.decode().split(',')
                if msg[0] == 'SCAN':
                    wait_for_sync = True; collecting = False
                    rotation_count = 0; point_batch = []
                    punti_totali_giro = 0; punti_scartati_vicini = 0

                    if len(msg) > 1:
                        target_giri = int(msg[1])
                   else:
                        target_giri = GIRI_DA_ACCUMULARE

                    print(f"\n[DEBUG RASPBERRY] Ricevuto SCAN. Giri richiesti: {target_giri}")

                elif msg[0] == 'M':
                    v_sx, v_dx, t = float(msg[1]), float(msg[2]), float(msg[3])
                    print(f"[DEBUG RASPBERRY] Motori: Sx={v_sx}, Dx={v_dx}, Tempo={t}s")
                    set_motors(v_sx, v_dx)
                    motor_stop_time = t_now + t
            except BlockingIOError: pass

            if wait_for_sync and scan.start_flag:
                wait_for_sync = False
                collecting = True
                continue

            if collecting:
                punti_totali_giro += 1

                if scan.distance > 50:
                    rad = np.deg2rad(360 - scan.angle)
                    d = scan.distance / 1000.0
                    point_batch.append(f"{d*np.cos(rad):.3f},{d*np.sin(rad):.3f}")
                else:
                    if scan.distance > 0:
                        punti_scartati_vicini += 1

                if len(point_batch) >= 45:
                    sock_out.sendto(";".join(point_batch).encode(), (MATLAB_IP, PORT_LIDAR))
                    point_batch = []

                if scan.start_flag:
                    rotation_count += 1
                    if rotation_count >= target_giri:
                        if point_batch: sock_out.sendto(";".join(point_batch).encode(), (MATLAB_IP, PORT_LIDAR))
                        sock_out.sendto(b"DONE", (MATLAB_IP, PORT_LIDAR))
                        collecting = False
                        print(f"[DEBUG RASPBERRY] Inviato 'DONE'. (Elaborati: {punti_totali_giro}, Scartati: {punti_scartati_vicini})")

    except Exception as e: print(f"Errore: {e}")
    finally:
        set_motors(0,0); lidar.stop(); lidar.set_motor_pwm(0); GPIO.cleanup()

if __name__ == "__main__": run()