from machine import I2C
from fpioa_manager import fm
import time
import math
import struct

# MPU9250 I2C Addresses
MPU_ADDR = 0x68
AK8963_ADDR = 0x0C

# Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
INT_PIN_CFG = 0x37
AK8963_CNTL1 = 0x0A
AK8963_ST1 = 0x02
AK8963_XOUT_L = 0x03

# I2C setup
fm.register(30, fm.fpioa.I2C0_SCLK)
fm.register(31, fm.fpioa.I2C0_SDA)
i2c = I2C(I2C.I2C0, freq=400000)

# Low-level I2C
def write_reg(addr, reg, val):
    i2c.writeto_mem(addr, reg, bytes([val]))

def read_reg(addr, reg, n=1):
    return i2c.readfrom_mem(addr, reg, n)

def read_word(addr, reg, little_endian=True):
    data = read_reg(addr, reg, 2)
    fmt = '<h' if little_endian else '>h'
    return struct.unpack(fmt, data)[0]

# Init sensors
write_reg(MPU_ADDR, PWR_MGMT_1, 0x00)
time.sleep_ms(100)
write_reg(MPU_ADDR, INT_PIN_CFG, 0x02)  # Bypass mode
write_reg(AK8963_ADDR, AK8963_CNTL1, 0x16)

# Mahony filter state
q = [1.0, 0.0, 0.0, 0.0]
twoKp = 2.0 * 0.5  # Proportional gain
twoKi = 2.0 * 0.0  # Integral gain
integralFB = [0.0, 0.0, 0.0]

def normalize(v):
    norm = math.sqrt(sum(i*i for i in v))
    return [i / norm for i in v] if norm != 0 else v

def read_sensors():
    # Accel
    ax = read_word(MPU_ADDR, ACCEL_XOUT_H, False) / 16384.0
    ay = read_word(MPU_ADDR, ACCEL_XOUT_H + 2, False) / 16384.0
    az = read_word(MPU_ADDR, ACCEL_XOUT_H + 4, False) / 16384.0

    # Gyro (in rad/s)
    gx = read_word(MPU_ADDR, GYRO_XOUT_H, False) / 131.0 * math.pi / 180
    gy = read_word(MPU_ADDR, GYRO_XOUT_H + 2, False) / 131.0 * math.pi / 180
    gz = read_word(MPU_ADDR, GYRO_XOUT_H + 4, False) / 131.0 * math.pi / 180

    # Mag
    if read_reg(AK8963_ADDR, AK8963_ST1)[0] & 0x01:
        mx = read_word(AK8963_ADDR, AK8963_XOUT_L, True)
        my = read_word(AK8963_ADDR, AK8963_XOUT_L + 2, True)
        mz = read_word(AK8963_ADDR, AK8963_XOUT_L + 4, True)
    else:
        mx = my = mz = 0

    return (ax, ay, az), (gx, gy, gz), (mx, my, mz)

def mahony_update(ax, ay, az, gx, gy, gz, mx, my, mz, dt):
    global q, integralFB

    q1, q2, q3, q4 = q
    # Normalize accelerometer
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0: return
    ax, ay, az = ax/norm, ay/norm, az/norm

    # Normalize magnetometer
    norm = math.sqrt(mx*mx + my*my + mz*mz)
    if norm == 0: return
    mx, my, mz = mx/norm, my/norm, mz/norm

    # Reference direction of Earth's magnetic field
    hx = 2*mx*(0.5 - q3*q3 - q4*q4) + 2*my*(q2*q3 - q1*q4) + 2*mz*(q2*q4 + q1*q3)
    hy = 2*mx*(q2*q3 + q1*q4) + 2*my*(0.5 - q2*q2 - q4*q4) + 2*mz*(q3*q4 - q1*q2)
    bx = math.sqrt((hx*hx) + (hy*hy))
    bz = 2*mx*(q2*q4 - q1*q3) + 2*my*(q3*q4 + q1*q2) + 2*mz*(0.5 - q2*q2 - q3*q3)

    # Estimated direction of gravity and magnetic field
    vx = 2*(q2*q4 - q1*q3)
    vy = 2*(q1*q2 + q3*q4)
    vz = q1*q1 - q2*q2 - q3*q3 + q4*q4
    wx = 2*bx*(0.5 - q3*q3 - q4*q4) + 2*bz*(q2*q4 - q1*q3)
    wy = 2*bx*(q2*q3 - q1*q4) + 2*bz*(q1*q2 + q3*q4)
    wz = 2*bx*(q1*q3 + q2*q4) + 2*bz*(0.5 - q2*q2 - q3*q3)

    # Error
    ex = (ay*vz - az*vy) + (my*wz - mz*wy)
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz)
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx)

    if twoKi > 0:
        integralFB[0] += twoKi * ex * dt
        integralFB[1] += twoKi * ey * dt
        integralFB[2] += twoKi * ez * dt
        gx += integralFB[0]
        gy += integralFB[1]
        gz += integralFB[2]
    else:
        integralFB = [0.0, 0.0, 0.0]

    gx += twoKp * ex
    gy += twoKp * ey
    gz += twoKp * ez

    # Integrate quaternion
    gx *= 0.5 * dt
    gy *= 0.5 * dt
    gz *= 0.5 * dt

    qa = q1
    qb = q2
    qc = q3
    q1 += -qb * gx - qc * gy - q4 * gz
    q2 += qa * gx + qc * gz - q4 * gy
    q3 += qa * gy - qb * gz + q4 * gx
    q4 += qa * gz + qb * gy - qc * gx

    norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
    q[:] = [q1 / norm, q2 / norm, q3 / norm, q4 / norm]

# Main loop
last = time.ticks_ms()
while True:
    acc, gyro, mag = read_sensors()
    now = time.ticks_ms()
    dt = (time.ticks_diff(now, last)) / 1000.0
    last = now

    mahony_update(*acc, *gyro, *mag, dt)
    print("Quat: w={:.3f}, x={:.3f}, y={:.3f}, z={:.3f}".format(*q))
    time.sleep_ms(50)
