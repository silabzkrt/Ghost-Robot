# Heterogeneous Real-Time Object Tracking System 

**A Hardware-in-the-Loop (HIL) vision system that distributes computational load between high-level software algorithms and low-level hardware logic.**

---

### 🎯 Project Overview
This project implements a robust object tracking system by bridging **Computer Vision** and **Embedded Logic**. Instead of relying on a single processor, the architecture splits the workload:
* **The "Brain" (MATLAB):** Handles computationally expensive tasks like image acquisition, segmentation, and Kalman Filtering.
* **The "Nervous System" (UART):** A custom serial protocol ensures low-latency data transmission.
* **The "Body" (Basys 3 FPGA):** Handles real-time I/O, driving a multiplexed 7-segment display and status LEDs via a custom Verilog Finite State Machine (FSM).

This hybrid approach demonstrates how to combine the flexibility of software with the deterministic timing of FPGA hardware.

### ⚙️ System Architecture
The system operates in a closed-loop cycle with a target refresh rate of **20Hz**.

1.  **Sensing:** MATLAB acquires frames from a live webcam.
2.  **Processing:** * **OpenCV/Vision Toolbox:** Color thresholding and centroid detection.
    * **Kalman Filter:** Fuses noisy measurements with a constant-velocity physics model to predict position during occlusion (glitch handling).
3.  **Communication:** Coordinates (X, Y) and Status flags are packed into a 4-byte packet and transmitted via USB-UART (9600 baud).
4.  **Actuation (Hardware):** The Artix-7 FPGA parses packets, updates the FSM state (SEARCHING vs. TRACKING), and drives the display.

---

### 🚀 Key Features

#### 🧠 Software (MATLAB)
* **Kalman Filter Implementation:** Custom state-space model ($A, H, Q, R$) to smooth tracking data and predict future trajectories.
* **Adaptive Segmentation:** Robust color thresholding to isolate target objects in varying lighting conditions.
* **Live Dashboard:** Real-time GUI logging velocity, acceleration, and motion history.

#### ⚡ Firmware (Verilog / Basys 3)
* **UART RX Module:** Custom asynchronous receiver logic to buffer incoming serial data.
* **Finite State Machine (FSM):** * `IDLE`: Scans for valid packet headers.
    * `TRACK`: Updates coordinates and drives Green LED.
    * `LOST`: Triggers Red LED and freezes display on last known position.
* **Multiplexed Display Driver:** Time-division multiplexing to drive 4-digit 7-segment displays using a single cathode bus.

---

### 🛠️ Hardware & Software Stack
* **Hardware:** Digilent Basys 3 (Artix-7 FPGA), Standard USB Webcam.
* **Software:** MATLAB R2024b (Computer Vision Toolbox, Instrument Control Toolbox).
* **HDL:** Verilog / VHDL (Xilinx Vivado).
* **Protocol:** UART (9600 Baud, 8N1).

---

### 📉 Performance Results
* **Tracking Accuracy:** ±2 pixels (Static), ±15 pixels (High-speed motion).
* **Occlusion Handling:** Successfully predicts path for up to 1.5 seconds of visual loss.
* **System Latency:** < 60ms (End-to-end from photon to FPGA LED).

---

### 👨‍💻 Usage

#### 1. Hardware Setup
1.  Connect the Basys 3 board to your PC via USB.
2.  Connect a webcam to your PC.
3.  Check Windows Device Manager to find the COM Port (e.g., `COM3`).

#### 2. FPGA Deployment
1.  Open the `/FPGA` folder in Xilinx Vivado.
2.  Generate Bitstream.
3.  Program the Basys 3 board.

#### 3. Software Execution
1.  Open MATLAB and navigate to the `/MATLAB` folder.
2.  Open `main_tracker.m`.
3.  Edit line 10 to match your COM port: `serialport("COM3", 9600)`.
4.  Run the script.
5.  Hold a red object in front of the camera to begin tracking.

---

### 📝 License
This project is open-source and available under the MIT License.
