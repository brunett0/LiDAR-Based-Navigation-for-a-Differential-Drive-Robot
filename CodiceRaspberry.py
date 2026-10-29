import math
import socket
import time
import RPi.GPIO as GPIO
from pyrplidar import PyRPlidar

# =========================================================================
# CONFIGURAZIONE SISTEMA EMBEDDED (RASPBERRY PI 4 - ROM3 ULTRA)
# =========================================================================
DEFAULT_MATLAB_IP  = "172.20.10.4"
PORT_LIDAR         = 5007
PORT_CONTROL       = 5005
GIRI_DA_ACCUMULARE = 5
K_RIGHT            = 1.08

# Parametri Riflesso di Emergenza Hardware On-Board (5000 Hz)
CONE_HALF_ANGLE_DEG = 10.0   # Cono frontale +/- 10 gradi
MIN_SAFE_DIST_MM    = 50.0   # 5 cm (ignora riflessi scocca)
MAX_DANGER_DIST_MM  = 200.0  # 20 cm (soglia di arresto immediato)
MIN_HITS_FOR_EMG    = 2      # Minimo raggi nel cono per evitare falsi positivi

# =========================================================================
# SETUP HARDWARE MOTORI E PONTE H
# =========================================================================
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
DIR_PINS = [20, 21, 26, 27, 22, 23, 24, 25]
PWM_PINS = [0, 13, 1, 12]

for p in DIR_PINS + PWM_PINS:
    GPIO.setup(p, GPIO.OUT)

pwms = [GPIO.PWM(p, 100) for p in PWM_PINS]
for p in pwms:
    p.start(0)

cur_v_sx = 0.0
cur_v_dx = 0.0


def set_motors(v_sx, v_dx):
    """Imposta velocità e direzione dei 4 motori (logica hardware validata ROM3 V35)."""
    global cur_v_sx, cur_v_dx
    cur_v_sx = v_sx
    cur_v_dx = v_dx

    v_dx_corr = v_dx * K_RIGHT
    p_sx = min(abs(v_sx) * 100.0, 100.0)
    p_dx = min(abs(v_dx_corr) * 100.0, 100.0)

    # Lato Sinistro
    GPIO.output(20, 0 if v_sx >= 0 else 1)
    GPIO.output(21, 1 if v_sx >= 0 else 0)
    GPIO.output(26, 1 if v_sx >= 0 else 0)
    GPIO.output(27, 0 if v_sx >= 0 else 1)

    # Lato Destro
    GPIO.output(22, 0 if v_dx >= 0 else 1)
    GPIO.output(23, 1 if v_dx >= 0 else 0)
    GPIO.output(24, 1 if v_dx >= 0 else 0)
    GPIO.output(25, 0 if v_dx >= 0 else 1)

    for i, val in enumerate((p_sx, p_sx, p_dx, p_dx)):
        try:
            pwms[i].ChangeDutyCycle(val)
        except Exception:
            pass


# =========================================================================
# SETUP SOCKET UDP NON-BLOCCANTE
# =========================================================================
sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_in.bind(("0.0.0.0", PORT_CONTROL))
sock_in.setblocking(False)

sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def run():
    matlab_ip = DEFAULT_MATLAB_IP
    lidar = PyRPlidar()

    try:
        while True:
            try:
                lidar.connect(port='/dev/ttyUSB0', baudrate=460800, timeout=3)
                lidar.set_motor_pwm(600)
                time.sleep(1.5)
                scan_gen = lidar.start_scan()
                print("ROM3 ULTRA - SERVER REAL-TIME CON RIFLESSO HARDWARE PRONTO.")

                collecting = False
                target_giri = GIRI_DA_ACCUMULARE
                accumulated_deg = 0.0
                prev_angle = None

                point_batch = []
                motor_stop_time = 0.0
                punti_totali_giro = 0
                punti_scartati_vicini = 0

                # Contatore per il riflesso di emergenza locale a 5000 Hz
                cone_danger_hits = 0
                emergency_triggered = False

                for scan in scan_gen():
                    t_now = time.monotonic()

                    # 1. Arresto temporizzato deterministico (clock monotonico)
                    if motor_stop_time > 0.0 and t_now >= motor_stop_time:
                        set_motors(0.0, 0.0)
                        motor_stop_time = 0.0
                        cone_danger_hits = 0

                    # 2. Svuotamento completo della coda comandi UDP (zero lag)
                    while True:
                        try:
                            data, addr = sock_in.recvfrom(1024)
                            if addr and addr[0]:
                                matlab_ip = addr[0]
                            msg = data.decode('utf-8', errors='ignore').strip().split(',')
                            cmd = msg[0].upper()

                            if cmd == 'SCAN':
                                collecting = True
                                accumulated_deg = 0.0
                                prev_angle = scan.angle
                                point_batch.clear()
                                punti_totali_giro = 0
                                punti_scartati_vicini = 0

                                if len(msg) > 1:
                                    try:
                                        target_giri = max(1, int(msg[1]))
                                    except ValueError:
                                        target_giri = GIRI_DA_ACCUMULARE
                                else:
                                    target_giri = GIRI_DA_ACCUMULARE

                                # Se è già scattata un'emergenza locale, notifica subito MATLAB
                                if emergency_triggered and target_giri == 1:
                                    sock_out.sendto(b"EMG", (matlab_ip, PORT_LIDAR))

                            elif cmd == 'M':
                                v_sx = float(msg[1])
                                v_dx = float(msg[2])
                                t_dur = float(msg[3])
                                set_motors(v_sx, v_dx)
                                if abs(v_sx) < 1e-4 and abs(v_dx) < 1e-4:
                                    motor_stop_time = 0.0
                                    emergency_triggered = False
                                    cone_danger_hits = 0
                                else:
                                    motor_stop_time = t_now + t_dur
                                    emergency_triggered = False
                                    cone_danger_hits = 0

                        except BlockingIOError:
                            break
                        except Exception as e_udp:
                            print(f"[WARN UDP] Pacchetto ignorato: {e_udp}")
                            break

                    # 3. RIFLESSO DI EMERGENZA LOCALE ON-BOARD (< 0.5 ms di latenza)
                    # Attivo solo durante l'avanzamento frontale (v_sx > 0 e v_dx > 0)
                    if scan.start_flag:
                        cone_danger_hits = 0

                    if motor_stop_time > 0.0 and cur_v_sx > 0.05 and cur_v_dx > 0.05:
                        if MIN_SAFE_DIST_MM < scan.distance < MAX_DANGER_DIST_MM:
                            ang_norm = (360.0 - scan.angle) % 360.0
                            if ang_norm <= CONE_HALF_ANGLE_DEG or ang_norm >= (360.0 - CONE_HALF_ANGLE_DEG):
                                cone_danger_hits += 1
                                if cone_danger_hits >= MIN_HITS_FOR_EMG:
                                    set_motors(0.0, 0.0)
                                    motor_stop_time = 0.0
                                    emergency_triggered = True
                                    cone_danger_hits = 0
                                    sock_out.sendto(b"EMG", (matlab_ip, PORT_LIDAR))
                                    print(f"[EMERGENZA ON-BOARD] Ostacolo a {scan.distance:.0f} mm! Arresto immediato.")

                    # 4. ACQUISIZIONE E TRASMISSIONE SCANSIONE SENZA ATTESA START_FLAG
                    if collecting:
                        punti_totali_giro += 1

                        # Calcolo angolo spazzato per contare esattamente target_giri * 360°
                        cur_angle = scan.angle
                        if prev_angle is not None:
                            delta_a = cur_angle - prev_angle
                            if delta_a < -180.0:
                                delta_a += 360.0
                            elif delta_a > 180.0:
                                delta_a -= 360.0
                            if delta_a > 0.0:
                                accumulated_deg += delta_a
                        prev_angle = cur_angle

                        if scan.distance > MIN_SAFE_DIST_MM:
                            rad = math.radians(360.0 - cur_angle)
                            d = scan.distance * 0.001
                            point_batch.append(f"{d * math.cos(rad):.3f},{d * math.sin(rad):.3f}")
                        elif scan.distance > 0.0:
                            punti_scartati_vicini += 1

                        if len(point_batch) >= 55:
                            sock_out.sendto(";".join(point_batch).encode('ascii'), (matlab_ip, PORT_LIDAR))
                            point_batch.clear()

                        # Completamento immediato al raggiungimento di 360° * target_giri
                        if accumulated_deg >= (360.0 * target_giri):
                            if point_batch:
                                sock_out.sendto(";".join(point_batch).encode('ascii'), (matlab_ip, PORT_LIDAR))
                                point_batch.clear()
                            sock_out.sendto(b"DONE", (matlab_ip, PORT_LIDAR))
                            collecting = False
                            if target_giri > 1:
                                print(f"[SCAN OK] {target_giri} giri inviati ({punti_totali_giro} pt, {punti_scartati_vicini} scartati)")

            except KeyboardInterrupt:
                raise
            except Exception as e_lidar:
                print(f"[WATCHDOG] Errore LiDAR ({e_lidar}). Ripristino automatico in 1s...")
                set_motors(0.0, 0.0)
                try:
                    lidar.stop()
                except Exception:
                    pass
                time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nChiusura richiesta dall'utente.")
    finally:
        set_motors(0.0, 0.0)
        try:
            lidar.stop()
            lidar.set_motor_pwm(0)
            lidar.disconnect()
        except Exception:
            pass
        GPIO.cleanup()


if __name__ == "__main__":
    run()
