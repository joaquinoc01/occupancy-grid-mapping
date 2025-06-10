# Occupancy Grid Mapping from Simulated Laser Scans

This project implements a **2D occupancy grid mapping algorithm** using simulated laser range scans, following the principles from *Probabilistic Robotics* by Thrun, Burgard, and Fox. A simulated robot moves along a **circular trajectory**, taking noisy LIDAR-like measurements at each pose. These scans are used to reconstruct the occupancy map from scratch using inverse sensor modeling and log-odds representation.

---

## 🚀 Features

- Simulated 2D world with a square-walled environment  
- Circular robot trajectory with inward-facing sensor  
- Synthetic trajectory and sensor data generation  
- Inverse sensor model using log-odds updates  
- Ray casting simulation for measurement generation  
- Visualization-ready CSV map output  
- Configurable parameters:  
  - Grid resolution  
  - Number of scan rays  
  - Log-odds parameters  
  - Noise injection  

---

## ✅ Fixes and Improvements

Early versions of the mapping pipeline had incorrect and noisy reconstructions. Key issues identified and resolved:

- **Bug: Degree-to-radian mismatch**  
  Scan angles were generated in degrees but interpreted as radians during ray casting and endpoint projection. This caused rays to be cast in incorrect directions, resulting in scattered and incorrect updates.

- **Fix: Generate angles in radians from the start**  
  The angles are now immediately converted to radians in `generateScanAngles()`, ensuring consistency throughout.

- **Improved clarity and robustness**:  
  - Replaced ambiguous probability thresholds with log-odds equivalents  
  - Clamped log-odds updates to prevent values from growing unbounded  

---

## 🛠️ Build & Run Instructions

This project uses C++ for the mapping core and Python for visualization. Make sure you have Eigen and Python 3 installed.

### 🔧 C++ Build

1. Create a build directory and compile:

    ```bash
    mkdir build && cd build
    cmake ..
    make
    ./occupancy_grid_mapping
    ```

   This generates `map.csv` in the `build/` directory, containing the occupancy probabilities.

---

### 🖼️ Python Visualization

2. Return to the project root and (optionally) create a virtual environment:

    ```bash
    python3 -m venv venv
    source venv/bin/activate  # On Windows: venv\Scripts\activate
    pip install matplotlib numpy
    ```

3. Run the visualization script:

    ```bash
    python show_map.py
    ```

   This reads `build/map.csv` and creates `build/map_visualized.png`.

---

## 🖼️ Screenshot


![Occupancy Map](https://github.com/user-attachments/assets/d3a20392-900c-44e8-8655-2c7ac6b3cb56)


---

## 📚 Reference

S. Thrun, W. Burgard, and D. Fox,  
*Probabilistic Robotics*, MIT Press, 1999.
