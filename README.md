# 1D Extended Kalman Filter Localization

This project implements a 1D Extended Kalman Filter in C++ using Eigen for tracking an object's position and velocity under noisy measurements and control input (acceleration). It follows concepts from Chapter 3 of *Probabilistic Robotics* by Thrun, Burgard, and Fox.

---

## 📦 Features

- Extended Kalman Filter for 1D motion
- Nonlinear motion model with acceleration
- Position-only measurement
- Uses Jacobians for prediction/update
- C++ with Eigen linear algebra library

---

## 📐 State Representation

```
State vector (x):
    [position, velocity]^T

Control input (u):
    [acceleration]

Measurement (z):
    [position]
```

---

## 🔁 Filter Cycle

Each time step includes:

1. **Prediction**
   - Apply motion model: `x' = f(x, u)`
   - Compute Jacobian of motion model `G`
   - Predict covariance: `P' = G * P * G^T + R`

2. **Update**
   - Predict measurement: `z' = h(x')`
   - Compute Jacobian of measurement model `H`
   - Compute Kalman gain: `K = P' * H^T * (H * P' * H^T + Q)^-1`
   - Update state: `x = x' + K * (z - z')`
   - Update covariance: `P = (I - K * H) * P'`

---

## 🗂️ Project Structure

```
.
├── CMakeLists.txt
├── include/
│   └── extended_kalman_filter.hpp
├── src/
│   ├── main.cpp
│   └── extended_kalman_filter.cpp
└── README.md
```

---

## 🛠️ Build Instructions

Ensure CMake and Eigen are installed:

```bash
git clone https://github.com/yourusername/1d-extended-kalman-filter-localization.git
cd 1d-extended-kalman-filter-localization
mkdir build && cd build
cmake ..
make
./kalman_filter
```

---

## 📈 Example Output

```
Step 0:
  True position:      1.05
  Noisy measurement:  0.866793
  Estimated position: 0.709718
  Estimated velocity: 0.414151

Step 1:
  True position:      2.2
  Noisy measurement:  3.3325
  Estimated position: 2.81948
  Estimated velocity: 1.44548

...

Step 9:
  True position:      15
  Noisy measurement:  13.3243
  Estimated position: 14.0272
  Estimated velocity: 1.54619
```

---

## 📚 References

- *Probabilistic Robotics* – Chapter 3  
  Sebastian Thrun, Wolfram Burgard, Dieter Fox

---

## 👤 Author

**Joaquín Ortega Cortés**  
📧 joaquinortegacortes@gmail.com
