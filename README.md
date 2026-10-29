# LiDAR-Based Navigation for a Differential Drive Robot

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%20%7C%20R2025b-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Python](https://img.shields.io/badge/Python-3.9%20%7C%203.11-blue.svg)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Hardware-Raspberry%20Pi%204%20%2B%20RPLIDAR%20C1-red.svg)](https://www.slamtec.com/en/Lidar/C1)
[![License](https://img.shields.io/badge/License-Academic%20Thesis-green.svg)]()

A distributed, high-precision robotics framework for autonomous mapping and navigation of a 4WD differential-drive robot (*Keyestudio KS0223* equipped with a *SLAMTEC RPLIDAR C1* and *Raspberry Pi 4*). 

The architecture decouples real-time on-board embedded actuation from high-level spatial computing via an ultra-low-latency asynchronous UDP client-server protocol.

---

## 🚀 Key Features & Architectural Improvements (v2.0)

1. **Dual-Map Architecture (A\* C-Space vs. Pure Wall Geometry)**
   - **`binMap_clean`**: Probabilistic occupancy grid with unknown space gated for safe A\* path planning, combined with morphological speckle filtering.
   - **`GLOBAL_WALLS`**: High-density 2 cm deduplicated true LiDAR point cloud extracted directly from the SLAM pose graph. Eliminates spurious exterior cells and resolves map-boundary distortions.
2. **Globally Optimal 2-Stage Scan-to-Map Localization + Kidnapped Recovery**
   - **Stage 1 (Correlative Grid Search)**: `matchScansGrid` with branch-and-bound optimization searches over $[\pm 0.45\text{ m}, \pm 45^\circ]$, rendering orientation tracking immune to local minima even under severe wheel slip.
   - **Stage 2 (Sub-Centimeter NDT)**: Fine continuous refinement via `matchScans` ($0.5\text{ m}$ cell size) achieving sub-centimeter ($< 3\text{ mm}$) and sub-degree ($< 0.2^\circ$) accuracy.
   - **Stage 3 (Automatic Anti-Kidnapping Recovery)**: Real-time geometric inlier overlap and trimmed RMSE validation. Automatically triggers a wide $[\pm 0.85\text{ m}, \pm 90^\circ]$ global search if inliers drop below threshold.
3. **Self-Healing Adaptive EKF (Joseph Form)**
   - Formulated with Joseph-stabilized covariance updates to guarantee positive definiteness.
   - Dynamic measurement noise $R(\text{RMSE}, \text{Inliers})$ and automated **Slip-Recovery** mechanism to absorb open-loop odometry slips without filter starvation.
   - Strictly enforced covariance floor ($P_{\min}$) preventing filter overconfidence during in-place rotations.
4. **Deterministic Stop & Go & Zero-Lag Embedded Control**
   - On-board hardware emergency reflex running at 5000 Hz (< 0.5 ms reaction time).
   - In-place pivot rotations executed cleanly without in-flight LiDAR packet collision or motion skewing.
   - Vectorized C-native UDP parsing (`sscanf`) reducing MATLAB CPU overhead to $< 0.5\text{ ms}$.

---

## 📂 Repository Structure

```text
├── CodiceRaspberry.py      # Embedded Python server (RPLIDAR C1 + 4WD PWM motor control)
├── Mapping.m               # Stage 1: EKF-SLAM mapping and odometry calibration benchmark
├── Navigation.m            # Stage 2: Autonomous A* navigation with Correlative EKF-ICP
├── deploy_raspberry.sh     # Network verification and deployment script for Raspberry Pi
├── sample_maps/            # Validated SLAM environment logs (.mat)
│   └── SLAM_Log_2026-10-25_20-58.mat
└── README.md
```

---

## 🛠️ Hardware Requirements & Pinout

- **Mobile Platform**: Keyestudio KS0223 4WD Robot Car
- **Embedded SBC**: Raspberry Pi 4 Model B (Raspberry Pi OS 64-bit)
- **LiDAR Sensor**: SLAMTEC RPLIDAR C1 (Direct ToF, 5000 Hz sample rate, 12m range) connected via USB (`/dev/ttyUSB0` at 460800 baud)
- **Power System**:
  - Motors: 2x 18650 Li-Ion batteries (7.4V nominal)
  - Raspberry Pi: Dedicated AZMUTH 5V / 3A portable power supply

### GPIO Pin Mapping (BCM Mode)
- **Motor Direction Pins**: `[20, 21, 26, 27, 22, 23, 24, 25]`
- **Motor PWM Speed Pins**: `[0, 13, 1, 12]`

---

## ⚡ Quick Start

### 1. On-Board Raspberry Pi Setup
```bash
# Clone the repository on the Raspberry Pi
git clone https://github.com/brunett0/LiDAR-Based-Navigation-for-a-Differential-Drive-Robot.git
cd LiDAR-Based-Navigation-for-a-Differential-Drive-Robot

# Install Python dependencies
pip install pyrplidar pyserial RPi.GPIO

# Run the on-board embedded server
python3 CodiceRaspberry.py
```

### 2. MATLAB Host Station
1. Connect your PC/Mac to the same local network / mobile hotspot as the Raspberry Pi (default target IP: `172.20.10.2`).
2. Run **`Mapping.m`**:
   - Press **`[N]`** to capture the zero-anchor scan (5 revolutions).
   - Click **`[G]`** on free space to explore and build the pose graph.
   - Press **`[S]`** to optimize the graph and export the standardized occupancy map (`SLAM_Log_*.mat`).
3. Run **`Navigation.m`**:
   - Load your pre-recorded `.mat` map.
   - Press **`[N]`** for high-confidence wide-angle global alignment ($\pm 90^\circ$).
   - Press **`[P]`** and click any destination point: the robot will autonomously plan with A\*, rotate to target headings, and drive forward with continuous scan-to-map EKF tracking.
   - Press **`[Q]`** to stop and display the 3-panel performance dashboard (CSV / MAT export).

---

## 🇮🇹 Descrizione del Progetto di Tesi

Il presente lavoro di tesi documenta la progettazione, l'implementazione e la validazione sperimentale di un'architettura hardware e software integrata e distribuita per la navigazione autonoma di un veicolo robotico mobile a trazione differenziale. L'infrastruttura si basa su un paradigma client-server operante in tempo reale tramite il protocollo di rete UDP a bassa latenza, strutturato per disaccoppiare il controllo embedded a basso livello — eseguito su una Single Board Computer Raspberry Pi 4 in linguaggio Python 3 — dall'elaborazione algoritmica ad alto livello, centralizzata in ambiente MATLAB su una stazione di controllo remota.

Sul fronte hardware, l'elaborato descrive la calibrazione cinematica della piattaforma e la riconfigurazione a basso livello del clock della UART del sistema operativo per consentire la comunicazione stabile con un sensore LiDAR omnidirezionale a tempo di volo diretto (D-ToF). La pipeline software di bordo sfrutta la programmazione asincrona e non bloccante per parallelizzare l'acquisizione dei dati laser e l'attuazione dei motori tramite modulazione PWM, azzerando i ritardi di sfasamento temporale. Per superare la degradazione geometrica delle scansioni indotta dal movimento, viene adottata e ottimizzata una strategia di navigazione discreta di tipo Stop & Go.

Il nucleo algoritmico affronta il problema della localizzazione e della mappatura simultanea. Al fine di mitigare la divergenza cumulativa della deriva odometrica e garantire la sostenibilità computazionale sulla piattaforma embedded, l'approccio EKF-SLAM completo viene impiegato esclusivamente nella prima macro-fase progettuale legata alla mappatura ed esplorazione dell'ambiente ignoto. Una volta generata e validata la mappa a griglia occupazionale statica, il paradosso della SLAM viene rimosso: nella successiva fase di navigazione autonoma, il framework software passa a una logica di localizzazione e tracking puro su mappa nota, disaccoppiando l'algoritmo di corrispondenza geometrica dall'espansione del vettore di stato del filtro probabilistico. All'interno di questo scenario di navigazione, viene implementata un'architettura di stima ibrida in cui coesistono il Filtro di Kalman Esteso (EKF) e l'algoritmo geometrico di Scan Matching correlativo (Branch-and-Bound). Questa fusione sensoristica viene governata da una logica di gating statistico basata sulla Distanza di Mahalanobis con distribuzione Chi-Quadro ($\chi^2$), specificamente progettata per identificare i massimi locali equiprobabili derivanti da ambienti simmetrici e neutralizzare l'effetto di scivolamento longitudinale tipico del problema dell'apertura (Aperture Problem). La pianificazione della traiettoria globale ottimale è infine affidata all'algoritmo A$^*$, integrato con una routine di Line-of-Sight Path Smoothing per la minimizzazione delle rotazioni fisiche, e protetto da un modulo reattivo di arresto di emergenza basato su un cono di sicurezza frontale monitorato in tempo di volo.

---

## 📜 Authors & Academic Citation
- **Author**: Matteo Pardo
- **Institution**: Università degli Studi Roma Tre - Dipartimento di Ingegneria
- **Thesis Title**: *Navigazione tramite sensore LiDAR per un Robot a Trazione Differenziale*
