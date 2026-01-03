# SynapTex: High-Fidelity Tactile Sensing Fabric

<div align="center">
[**Firmware**](#firmware) | [**Visualizer**](#visualization) | [**Simulation**](#mujoco-simulation)

</div>

## 📖 Introduction (项目简介)

**SynapTex** is an advanced tactile sensing system designed for flexible fabrics. It provides high-resolution real-time pressure monitoring with minimized crosstalk. This project integrates an embedded firmware solution (Arduino) with a robust Python-based visualization toolkit, supporting both real-world hardware and MuJoCo simulations.

Key features:
- **Adaptive Calibration**: Algorithms optimized for different contact surface areas.
- **Real-time Visualization**: Multi-thread processing for low-latency feedback.
- **Simulation Ready**: Full support for MuJoCo physics engine integration.

---

## 🛠️ Hardware & Firmware (固件配置)

The firmware is designed to run on Arduino-compatible microcontrollers. We provide three specialized versions to suit different experimental needs:

| Version                  | Description                                                  | Recommended Use Case                                    |
| :----------------------- | :----------------------------------------------------------- | :------------------------------------------------------ |
| **`MatrixArray_naive`**  | Raw data output without calibration.                         | Debugging hardware connections.                         |
| **`MatrixArray_normal`** | **Standard Calibration**. Balances sensitivity and crosstalk. | **General Purpose** (Large contact areas).              |
| **`MatrixArray_update`** | **Precision Calibration**. High sensitivity with minimal crosstalk. | Fine-grained sensing (Small objects) & **Simulations**. |

> **Setup**: Upload the `.ino` file corresponding to your use case to the MCU.

---

## 💻 Software Environment (软件环境)

### 1. Prerequisites

We recommend managing dependencies via Conda to avoid conflicts.

```python
# Create and activate environment
conda create --name SynapTex python=3.10
conda activate SynapTex

# Install core dependencies
pip install pyserial opencv-python==4.6.0.66 scipy numpy==1.23.0 mujoco==3.3.0
```

### 2. Visualization (Real-world Testing)

To visualize data from the tactile fabric:

1. Connect the hardware to your PC.
2. Ensure `MatrixArray_normal` (or compatible firmware) is running.
3. Execute the multi-thread visualizer:

Bash

```
python3 python/real/multi_thread_contact_v0.py
```

#### ⚙️ Matrix Mapping Configuration

Depending on your fabric version (Standard vs. Customized), you may need to adjust the data mapping logic in the Python script.

- **Standard Version**: Uses sequential mapping (Default).
- **Customized Version**: Requires row re-ordering (lines 8-15).

> *Check the comments in `multi_thread_contact_v0.py` to toggle between Standard and Customized mapping modes.*

------

## 🤖 MuJoCo Simulation (仿真测试)

SynapTex includes a simulation bridge to test tactile algorithms in a virtual environment before deploying to hardware.

Bash

```
cd python/sim
python3 sim_touch_vis.py
```

*Note: Ensure your XML file paths in the script match your local directory structure.*

------

## 🔧 Troubleshooting & Configuration

### Serial Port Setup

Modify the port variable in the Python scripts according to your OS:

- **Linux**: `/dev/ttyUSB*` (e.g., `/dev/ttyUSB1`)
- **Windows**: `COM*` (e.g., `COM6`)

### Noise & Thresholding

Adjust the visualization sensitivity in `multi_thread_contact_v0.py` if the signal is too noisy:

Python

```
THRESHOLD = 6       # Minimum pressure value to register a touch
NOISE_SCALE = 20    # Gain factor for visualization
```

------

## 📜 Citation & Acknowledgements

This project builds upon the foundational work of 3D-ViTac. If you use SynapTex in your research, please cite:

代码段

```
@misc{jnu2025SynapTex,
  title={SynapTex: A Smart Tactile Sensing Fabric},
  author={Ma, Jun and Yang},
  year={2025},
  howpublished={\url{[https://github.com/1250554826/SynapTex](https://github.com/1250554826/SynapTex)}},
}
```

**References:**

- [3D-ViTac Project](https://binghao-huang.github.io/3D-ViTac/)

