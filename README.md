# MPU9250 Orientation with MaixPy (Mahony Filter)

This project demonstrates how to read **accelerometer**, **gyroscope**, and **magnetometer** data from the **MPU9250** directly via I²C on the **Kendryte K210 (Maix M1 Dock)**, compute orientation using the **Mahony filter**, and output **quaternion** values — **without using any external libraries**.

---

## Files

- `main.py`: Full MaixPy script
  - I²C communication with MPU9250
  - Reads raw sensor data
  - Implements Mahony filter to calculate orientation quaternion

---

## Requirements

-  **Hardware**:
  - Maix M1 Dock (K210)
  - MPU9250 sensor module
  - I²C connected (default: `IO30 = SCL`, `IO31 = SDA`)

-  **Software**:
  - [MaixPy firmware](https://maixpy.sipeed.com/)
  - Flash tool: `kflash_gui` or `kflash.py`
  - Serial tool: `maixpy-ide`, `screen`, or similar

---

## 🛠️ Wiring (Default Pinout)

| MPU9250 | K210 (Maix M1) |
|---------|----------------|
| VCC     | 3.3V           |
| GND     | GND            |
| SDA     | IO31           |
| SCL     | IO30           |

> **Note**: Pull-up resistors (4.7kΩ) on SDA/SCL are recommended.

---

## Usage

1. Connect your MPU9250 to the K210.
2. Flash the `main.py` script to your board using `maixpy-ide` or `kflash_gui`.
3. Open the serial terminal.
4. You should see real-time quaternion orientation output:
   ```
   Quat: w=0.991, x=0.005, y=0.060, z=-0.120
   ```

---

## 🔄 Output Format

The output is a **quaternion** representing 3D orientation:

```
Quat: w=..., x=..., y=..., z=...
```

This can be used for:
- 3D head tracking
- Motion capture
- IMU data fusion
- Robotics and control systems

---

## Notes

- Sensor scaling is configured for:
  - ±2g acceleration
  - ±250°/s gyroscope
- Mahony filter uses simple gains:
  - `Kp = 0.5`
  - `Ki = 0.0` (no integral correction)
- Filter runs at ~20Hz

---

## References

- [MPU9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf)
- [Mahony Filter Algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
