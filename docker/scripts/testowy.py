import numpy as np
from scipy.optimize import differential_evolution, minimize
from scipy.spatial.transform import Rotation as R_scipy

# dane wejściowe
prev_imu_vec = np.array([15.324, -22.071, 0.000])
imu_vec = np.array([15.482, -22.508, 0.000])

# normalizuj wektory IMU tylko raz
imu_vec = imu_vec / np.linalg.norm(imu_vec)
prev_imu_vec = prev_imu_vec / np.linalg.norm(prev_imu_vec)

# wektory z VO (normalizowane, jeśli porównujesz kierunki)
prev_vo_pose = np.array([207.46203653903274, 15.74735786712581, 353.188669538078])
vo_pose = np.array([207.79257903725193, 15.824206609477967, 355.1083368444674])
norma = np.linalg.norm(vo_pose)
vo_pose = vo_pose / norma
prev_vo_pose = prev_vo_pose / np.linalg.norm(prev_vo_pose)

d_imu = imu_vec - prev_imu_vec
d_vo = vo_pose - prev_vo_pose

d_imu = d_imu / np.linalg.norm(d_imu)
d_vo = d_vo / np.linalg.norm(d_vo)

def fun(x):  # x = [roll, pitch, yaw] (radiany), Z-Y-X (zyx)
    c_phi, s_phi = np.cos(x[0]), np.sin(x[0])
    c_theta, s_theta = np.cos(x[1]), np.sin(x[1])
    c_psi, s_psi = np.cos(x[2]), np.sin(x[2])

    # Macierz R (Z-Y-X)
    R_00 = c_theta * c_psi
    R_01 = c_theta * s_psi
    R_02 = -s_theta

    R_10 = s_phi * s_theta * c_psi - c_phi * s_psi
    R_11 = s_phi * s_theta * s_psi + c_phi * c_psi
    R_12 = s_phi * c_theta

    R_20 = c_phi * s_theta * c_psi + s_phi * s_psi
    R_21 = c_phi * s_theta * s_psi - s_phi * c_psi
    R_22 = c_phi * c_theta

    R = np.array([
        [R_00, R_01, R_02],
        [R_10, R_11, R_12],
        [R_20, R_21, R_22]
    ])

    pred_pose = R @ vo_pose
    pred_gorth = R @ d_vo

    e_pose = np.sum((imu_vec - pred_pose) ** 2)
    e_groth = np.sum((d_imu - pred_gorth) ** 2)
    return e_pose + e_groth

# optymalizacja globalna (DE) + lokalne dopracowanie (BFGS)
bounds = [(-np.pi, np.pi), (-np.pi/2, np.pi/2), (-np.pi, np.pi)]
res = differential_evolution(fun, bounds, seed=42)
print("DE success:", res.success)
print("DE angles (rad):", res.x)

res_local = minimize(fun, res.x, method='BFGS')
x_final = res_local.x if res_local.success else res.x
print("Local refine success:", res_local.success)
print("Final angles (rad):", x_final)
print("Final cost:", fun(x_final))

# zbuduj macierz R z x_final i policz predykcję
c_phi, s_phi = np.cos(x_final[0]), np.sin(x_final[0])
c_theta, s_theta = np.cos(x_final[1]), np.sin(x_final[1])
c_psi, s_psi = np.cos(x_final[2]), np.sin(x_final[2])

R_mat = np.array([
    [c_theta * c_psi,                c_theta * s_psi,              -s_theta],
    [s_phi * s_theta * c_psi - c_phi * s_psi,  s_phi * s_theta * s_psi + c_phi * c_psi,  s_phi * c_theta],
    [c_phi * s_theta * c_psi + s_phi * s_psi,  c_phi * s_theta * s_psi - s_phi * c_psi,  c_phi * c_theta]
])

pred_imu = R_mat @ vo_pose
pred_imu = pred_imu * norma
print("Predykcja wektora IMU po rotacji VO:", pred_imu)

# kwaternion z macierzy (x, y, z, w)
r = R_scipy.from_matrix(R_mat)
quaternion = r.as_quat()
print("-" * 40)
print("Wynikowy kwaternion (x, y, z, w):")
print(f"x: {quaternion[0]:.6f}")
print(f"y: {quaternion[1]:.6f}")
print(f"z: {quaternion[2]:.6f}")
print(f"w: {quaternion[3]:.6f}")