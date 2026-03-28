import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
import calculos as calculos


def compare_positions(gt_csv='tello_ground_truth.csv', imu_csv='tello_imu_example.csv', out_dir='out'):
    os.makedirs(out_dir, exist_ok=True)

    # Load ground truth
    with open(gt_csv, 'r', newline='', encoding='utf-8') as f:
        gt_rows = list(csv.DictReader(f))

    gt_times = np.array([float(r['time_s'] if 'time_s' in r else r.get('time', r['time'])) for r in gt_rows], dtype=float)
    gt_x = np.array([float(r['gt_pos_x_m']) for r in gt_rows], dtype=float)
    gt_y = np.array([float(r['gt_pos_y_m']) for r in gt_rows], dtype=float)
    gt_z = np.array([float(r['gt_pos_z_m']) for r in gt_rows], dtype=float)

    # Load IMU and integrate
    with open(imu_csv, 'r', newline='', encoding='utf-8') as f:
        imu = list(csv.DictReader(f))

    time_list, _, P_ned, _, _, _ = calculos.integrate_imu_data(imu)
    imu_times = np.array([float(t) for t in time_list], dtype=float)

    # Ensure P_ned is an array of shape (N,3)
    P_arr = np.array([np.asarray(p).astype(float) for p in P_ned])
    if P_arr.ndim == 1 and P_arr.size == 3:
        # Single sample
        P_arr = P_arr.reshape((1, 3))

    # Interpolate estimated trajectory to ground-truth timestamps per axis
    est_x = np.interp(gt_times, imu_times, P_arr[:, 0])
    est_y = np.interp(gt_times, imu_times, P_arr[:, 1])
    est_z = np.interp(gt_times, imu_times, P_arr[:, 2])

    # Compute percent errors safely
    eps = 1e-8
    def pct_err(est, gt):
        err = np.empty_like(est)
        err[:] = np.nan
        mask = np.abs(gt) > eps
        err[mask] = (est[mask] - gt[mask]) / gt[mask] * 100.0
        return err

    err_x_pct = pct_err(est_x, gt_x)
    err_y_pct = pct_err(est_y, gt_y)
    err_z_pct = pct_err(est_z, gt_z)

    # Norm percent error (euclidean)
    gt_norm = np.linalg.norm(np.vstack((gt_x, gt_y, gt_z)).T, axis=1)
    est_norm = np.linalg.norm(np.vstack((est_x, est_y, est_z)).T, axis=1)
    err_norm_pct = np.full_like(est_norm, np.nan)
    mask = gt_norm > eps
    err_norm_pct[mask] = (est_norm[mask] - gt_norm[mask]) / gt_norm[mask] * 100.0

    # Summary statistics
    def stats(arr):
        valid = arr[~np.isnan(arr)]
        if valid.size == 0:
            return {'mean': np.nan, 'median': np.nan, 'std': np.nan, 'max': np.nan}
        return {'mean': float(np.mean(valid)), 'median': float(np.median(valid)), 'std': float(np.std(valid)), 'max': float(np.max(np.abs(valid)))}

    stats_x = stats(err_x_pct)
    stats_y = stats(err_y_pct)
    stats_z = stats(err_z_pct)
    stats_norm = stats(err_norm_pct)

    # Save per-step CSV
    out_csv = os.path.join(out_dir, 'comparison_results.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'gt_x_m', 'est_x_m', 'err_x_pct', 'gt_y_m', 'est_y_m', 'err_y_pct', 'gt_z_m', 'est_z_m', 'err_z_pct', 'err_norm_pct'])
        for i in range(len(gt_times)):
            writer.writerow([
                gt_times[i],
                gt_x[i], est_x[i], err_x_pct[i] if not math.isnan(err_x_pct[i]) else '',
                gt_y[i], est_y[i], err_y_pct[i] if not math.isnan(err_y_pct[i]) else '',
                gt_z[i], est_z[i], err_z_pct[i] if not math.isnan(err_z_pct[i]) else '',
                err_norm_pct[i] if not math.isnan(err_norm_pct[i]) else ''
            ])

    # Plots
    # Trajectories over time
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    ax[0].plot(gt_times, gt_x, label='gt_x')
    ax[0].plot(gt_times, est_x, label='est_x', linestyle='--')
    ax[0].set_ylabel('X (m)')
    ax[0].legend()

    ax[1].plot(gt_times, gt_y, label='gt_y')
    ax[1].plot(gt_times, est_y, label='est_y', linestyle='--')
    ax[1].set_ylabel('Y (m)')
    ax[1].legend()

    ax[2].plot(gt_times, gt_z, label='gt_z')
    ax[2].plot(gt_times, est_z, label='est_z', linestyle='--')
    ax[2].set_ylabel('Z (m)')
    ax[2].set_xlabel('time_s')
    ax[2].legend()

    fig.suptitle('Ground Truth vs Estimated Positions')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    traj_png = os.path.join(out_dir, 'trajectory_comparison.png')
    fig.savefig(traj_png)
    plt.close(fig)

    # Error plots
    fig2, ax2 = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    ax2[0].plot(gt_times, err_x_pct, label='err_x_pct')
    ax2[0].set_ylabel('err_x_pct')
    ax2[0].legend()

    ax2[1].plot(gt_times, err_y_pct, label='err_y_pct')
    ax2[1].set_ylabel('err_y_pct')
    ax2[1].legend()

    ax2[2].plot(gt_times, err_z_pct, label='err_z_pct')
    ax2[2].set_ylabel('err_z_pct')
    ax2[2].legend()

    ax2[3].plot(gt_times, err_norm_pct, label='err_norm_pct')
    ax2[3].set_ylabel('err_norm_pct')
    ax2[3].set_xlabel('time_s')
    ax2[3].legend()

    fig2.suptitle('Percent Errors over Time')
    fig2.tight_layout(rect=[0, 0, 1, 0.96])
    err_png = os.path.join(out_dir, 'errors_over_time.png')
    fig2.savefig(err_png)
    plt.close(fig2)

    # Print summary
    print('Summary statistics (percent):')
    print('X:', stats_x)
    print('Y:', stats_y)
    print('Z:', stats_z)
    print('Norm:', stats_norm)
    print(f'Per-step CSV saved to: {out_csv}')
    print(f'Trajectory plot saved to: {traj_png}')
    print(f'Error plot saved to: {err_png}')


if __name__ == '__main__':
    compare_positions()
    