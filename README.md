# Vision-Controlled Robotic Hand

A fully functional robotic hand controlled in real time using computer vision.  
The system tracks human hand motion using OpenCV and MediaPipe and mirrors finger movements on a physical robotic hand via an Arduino-based control stack.

This project has been physically built, powered, tested, and demonstrated. It is not a simulation or proof-of-concept.

---

## Project Overview

The robotic hand replicates human finger motion by combining real-time hand landmark detection with embedded servo control.

A camera feed is processed on a host computer using MediaPipe Hands. Finger joint positions are converted into joint angles, which are streamed over a serial interface to an Arduino Nano. The Arduino generates PWM signals to drive individual servo motors controlling each finger.

The mechanical structure is based on the open-source InMoov hand design. All electronics, wiring, power distribution, and control logic were designed and implemented independently.

---

## System Architecture

Camera  
→ OpenCV + MediaPipe (Python)  
→ Finger angle computation  
→ Serial communication  
→ Arduino Nano  
→ Servo PWM output  
→ Robotic hand motion

---

## Key Features

- Real-time vision-based finger tracking  
- No gloves, flex sensors, or wearable hardware  
- Arduino Nano used as a dedicated low-level controller  
- High-current external power supply for servos  
- Stable operation under continuous use  
- Modular wiring and serviceable electronics layout  

---

## Mechanical Design

- Mechanical base: InMoov robotic hand (open-source)
- Actuation:
  - 5 × MG996R high-torque servo motors (one per finger)
- Tendon system:
  - Nylon tendons routed through printed finger segments
- Wrist joint:
  - Experimentally tested and later locked to avoid excessive tendon stress

Mechanical modifications were made to integrate electronics, power routing, and cable management into the palm and forearm structure.

---

## Electronics and Power

The control and power system is implemented using a zero board (perfboard) with point-to-point wiring.

A custom PCB was intentionally not used for this build. Given the prototype-oriented nature of the system and the need for rapid iteration, a zero board solution provided faster debugging, easier modifications, and sufficient reliability.

### Microcontroller
- Arduino Nano
  - Dedicated to low-level servo control
  - Receives real-time angle commands over serial

### Power System
- High-current DC–DC buck converter
- External LiPo battery supply
- Separate high-current servo power rail
- Common ground shared between logic and power stages

### Wiring
- Servo power routed using thicker-gauge wiring
- Signal lines kept short to reduce noise
- All interconnections soldered directly on perfboard

This approach proved stable under continuous operation and allowed quick changes during calibration and testing.

---

## Computer Vision Pipeline

- Libraries:
  - OpenCV
  - MediaPipe Hands
  - PySerial
- Processing steps:
  1. Hand detection
  2. Extraction of finger joint landmarks
  3. Computation of finger curl angles
  4. Normalization and range mapping
  5. Smoothing and rate limiting
  6. Transmission to Arduino via serial

Filtering is applied to reduce jitter and prevent abrupt servo movements.

---

## Serial Communication Protocol

Commands sent from the host computer to the Arduino use a simple text-based protocol.

Example format:
S:thumb,index,middle,ring,pinky


Example command:
S:90,45,78,80,120


On communication loss or invalid input, the controller holds the last valid state or moves to a safe open-hand position.

---

## Calibration and Testing

- Per-finger calibration for minimum, maximum, and neutral positions
- Software-side clamping to prevent mechanical overtravel
- Tested under continuous operation
- Observed latency: approximately 300 ms end-to-end

The system remains stable during prolonged operation, with predictable servo behavior.

---

## Known Limitations

- No force or tactile feedback
- Wrist articulation disabled for reliability
- Performance depends on lighting conditions
- Tendon wear over extended use (expected for nylon systems)

These limitations are documented and understood based on physical testing.

---

## Licensing

- Hardware design files: CERN Open Hardware Licence v2
- Software: MIT License
- Mechanical base design: InMoov project by Gaël Langevin

Original electronics design, control logic, and system integration are independently implemented.

---

## Academic Context

Developed as part of Project-Based Learning (PBL)  
Manipal University Jaipur

---

## Author

Pranav Anil

---

## Notes

This repository documents a working robotic system.  
It is intended for learning, experimentation, and reproducibility rather than marketing or demonstration-only purposes.


