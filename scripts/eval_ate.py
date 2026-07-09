# SLAM 궤적 ATE 평가 — 스파스 GT 컨트롤 포인트 대비 절대 궤적 오차.
# SLAM 월드는 임의 원점/방향이므로 Umeyama 강체 정렬 후 잔차를 잰다.
# 사용: python eval_ate.py <slam_trajectory.tum> <gt.txt(TUM 형식)>
import sys

import numpy as np
from scipy.interpolate import interp1d


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mu_d - R @ mu_s


def main(traj_path, gt_path):
    sl = np.loadtxt(traj_path)
    gt = np.loadtxt(gt_path)
    fp = interp1d(sl[:, 0], sl[:, 1:4], axis=0, bounds_error=False)
    est = fp(gt[:, 0])
    ok = np.isfinite(est[:, 0])
    if ok.sum() < 3:
        sys.exit(f'시간 겹침 부족: {ok.sum()}개')
    g, e = gt[ok, 1:4], est[ok]
    R, t = umeyama(e, g)
    res = np.linalg.norm((R @ e.T).T + t - g, axis=1)
    print(f'GT 포인트 {ok.sum()}/{len(gt)}개 | ATE 평균 {res.mean():.3f}m  '
          f'중앙값 {np.median(res):.3f}m  최대 {res.max():.3f}m')
    for i, r in zip(np.where(ok)[0], res):
        bar = '#' * int(r * 10)
        print(f'  t={gt[i, 0] - gt[0, 0]:6.1f}s  err={r:5.2f}m  {bar}')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit('사용: python eval_ate.py <slam.tum> <gt.txt>')
    main(sys.argv[1], sys.argv[2])
