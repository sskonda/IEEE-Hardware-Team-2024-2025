import numpy as np
from pathlib import Path

# ---------- Fit ellipsoid ----------
def fit_ellipsoid(data):
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    D = np.stack([x**2, y**2, z**2, x, y, z, np.ones_like(x)], axis=1)
    u, _, _, _ = np.linalg.lstsq(D, np.ones_like(x), rcond=None)
    A, B, C, D_, E, F, G = u
    Q = np.array([[A, 0, 0, D_/2],
                  [0, B, 0, E/2],
                  [0, 0, C, F/2],
                  [D_/2, E/2, F/2, G - 1]])
    Q3 = Q[:3, :3]
    q = -np.linalg.solve(Q3, u[3:6] / 2)
    val = -1.0  # Normalize
    a = np.sqrt(abs(val / A))
    b = np.sqrt(abs(val / B))
    c = np.sqrt(abs(val / C))
    return {'center': q, 'axes': (a, b, c)}

# ---------- Normalize ----------
def normalize_ellipsoid(data, center, axes):
    return (data - center[None, :]) / axes[None, :]

# ---------- Load IMU data ----------
data = np.load('imu_calibration_data.npy')
accel_data = data[:, 0:3]
gyro_data = data[:, 3:6]

# ---------- Global ellipsoid fit (accelerometer and gyro) ----------
accel_fit = fit_ellipsoid(accel_data)

accel_center = np.array(accel_fit['center'])
accel_axes = np.array(accel_fit['axes'])

gyro_center = np.mean(gyro_data, axis=0)
# gyro_axes = np.std(gyro_data, axis=0)

print("Global Accelerometer Ellipsoid Fit:")
print("  Center:", accel_center)
print("  Axes:", accel_axes)

print("\nGlobal Gyroscope Ellipsoid Fit:")
print("  Center:", gyro_center)
# print("  Axes:", gyro_axes)

# ---------- Pose-wise covariance using global fit ----------
num_poses = 15
samples_per_pose = 10000

for i in range(num_poses):
    print(f"\n=== Pose {i+1} ===")

    start = i * samples_per_pose
    end = (i + 1) * samples_per_pose
    accel_block = accel_data[start:end]
    gyro_block = gyro_data[start:end]

    # Normalize using global ellipsoid fit
    accel_norm = normalize_ellipsoid(accel_block, accel_center, accel_axes)
    gyro_norm = gyro_block - gyro_center

    # Fix magnitude scaling (rescale to unit sphere)
    accel_mag = np.linalg.norm(accel_norm, axis=1)
    accel_norm_fixed = accel_norm / np.mean(accel_mag)

    # Convert to m/s²
    accel_mps2 = accel_norm_fixed * 9.80665

    # Covariance
    accel_cov = np.cov(accel_mps2, rowvar=False)
    gyro_cov = np.cov(gyro_norm, rowvar=False)

    # Magnitudes
    accel_mean_mag_g = np.mean(np.linalg.norm(accel_norm_fixed, axis=1))
    accel_mean_mag_mps2 = np.mean(np.linalg.norm(accel_mps2, axis=1))
    gyro_mean_mag = np.mean(np.linalg.norm(gyro_norm, axis=1))

    # Output
    np.set_printoptions(precision=6, suppress=True)
    print("Accelerometer Covariance (m/s²):\n", accel_cov)
    print("Accelerometer Mean Magnitude (G):", accel_mean_mag_g)
    print("Accelerometer Mean Magnitude (m/s²):", accel_mean_mag_mps2)
    print("Gyroscope Covariance:\n", gyro_cov)
    print("Gyroscope Mean Magnitude:", gyro_mean_mag)

# Save calibration data
path = Path('~/.ros/gyro_info/').expanduser()
path.mkdir(parents=True, exist_ok=True)
np.savez(
    str(path.joinpath('imu_calibration.npz').resolve()),
    angular_offset=gyro_center * (np.pi / 180.0) / 131.0,
    linear_offset=accel_center,
    # angular_scale=np.ones(3, dtype=np.float64),
    # linear_scale=accel_axes,
    angular_covariance=np.cov(gyro_data * (np.pi / 180.0) / 131.0, rowvar=False).flatten(),
)