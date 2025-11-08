# posturePal02

# 🐶 Posture Pal

**Posture Pal** is an interactive, posture-correcting desk companion created for **BoxBots 2025**, a hardware hackathon focused on robotics and human-centered design.

Teammates: Sonya Qu, Ariel Liu, Mansi Katarey, Alexandra

Using **OpenCV**, **Python**, and **Arduino**, it monitors your posture in real time and responds through a cardboard robotic pet that reacts to your body alignment, gently encouraging you to sit up straight.

---

## ✨ Features

- 🧍‍♀️ Real-time posture detection using your webcam (OpenCV)
- 🎯 Automatic calibration of your “good posture” in the first 60 seconds
- 📊 Monitors spine curvature and head tilt (left-right and forward-back)
- 💡 Reacts every 30 frames — gentle reminders rather than constant nagging
- 🐕 Physical feedback through servos and sound:
  - **GOOD posture:** ears upright, tail wagging fast  
  - **WARN posture:** ears slightly down, tail wagging slowly  
  - **BAD posture:** ears down completely, faint barking sound, tail stops moving 

---

## 🧠 How It Works

### 1. Posture Tracking (Python + OpenCV)
- Captures webcam frames.
- Calibrates your **“good posture baseline”** during the first 60 seconds.
- Tracks:
  - Spine curvature
  - Head tilt (left/right, forward/back)
- Every 30 frames, it checks if you’ve deviated from your baseline.
- If you stay off-baseline for 60 frames:
  - `GOOD` → upright posture  
  - `WARN` → mild slouch or tilt  
  - `BAD` → deeper slouch or lean  

The posture state is sent to the Arduino via **Serial communication**.

---

### 2. Arduino Feedback (Hardware)
- The Arduino listens for posture updates (`GOOD`, `WARN`, or `BAD`) via Serial.
- It controls:
  - A **servo motor** for the **ears**
  - A **servo motor** for the **tail**
  - A **buzzer** (for soft barking when posture is bad)

| Posture | Ears | Tail | Sound |
|----------|------|------|-------|
| GOOD | Upright | Wags fast | — |
| WARN | Slightly down | Wags slowly | — |
| BAD | Fully down | Stops | Soft bark |

---

## 🧰 Tech Stack

| Component | Technology |
|------------|-------------|
| Computer Vision | Python + OpenCV |
| Pose Estimation | MediaPipe (or OpenCV landmarks) |
| Microcontroller | Arduino Uno / Nano |
| Hardware | MG90S Micro Servos, Buzzer |
| Chassis | Cardboard + 3D printed mounts |