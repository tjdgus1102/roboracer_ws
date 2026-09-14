#!/usr/bin/env python3
"""run_comparison.sh 가 남긴 기록에서 컨트롤러 비교표를 뽑는다.

    python3 tools/compare_laps.py [clean|obs]

랩 구분은 record_lap 과 같은 방식이다. 주행 x,y 를 레이스라인에 투영해
진행거리 s 를 만들고, s 가 한 바퀴만큼 튀는 지점을 시작선 통과로 본다.
트랙 길이의 90% 이상을 덮은 구간만 랩으로 인정한다 (부분 아웃랩 제외).
"""
import glob
import os
import re
import sys

import numpy as np
import yaml
from PIL import Image

WS = os.path.expanduser('~/roboracer_ws')
OBSTACLES = [(11, 8), (10.5, 6.5), (9.5, 7.5), (8.5, 4.5), (7.5, 5.5), (7, 4)]
OBS_RADIUS = 0.065
# 컨트롤러마다 "못 풀었다" 를 알리는 문자열이 다르다
FAIL_PATTERNS = {
    'mppi': 'Optimizer fail to compute path',
    'rpp': 'detected collision ahead',
    'dwb': 'No valid trajectories',
}
MISS = 'Control loop missed its desired rate'


def laps_of(t, xy, ref, spacing):
    idx = np.argmin(np.linalg.norm(xy[:, None, :] - ref[None, :, :], axis=2), axis=1)
    s = idx * spacing
    track_len = spacing * len(ref)
    gap = np.where(np.abs(np.diff(s)) > track_len / 2)[0]
    bounds = np.concatenate(([0], gap + 1, [len(s)]))
    out = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        if np.ptp(s[a:b]) > 0.9 * track_len:
            out.append(t[b - 1] - t[a])
    return out


def stops(t, v, thresh=0.1, min_dur=0.5):
    """정지 구간: 속도가 thresh 미만으로 min_dur 이상 유지된 구간."""
    slow = v < thresh
    n, total = 0, 0.0
    k = 0
    while k < len(slow):
        if slow[k]:
            j = k
            while j + 1 < len(slow) and slow[j + 1]:
                j += 1
            dur = t[j] - t[k]
            if dur >= min_dur:
                n += 1
                total += dur
            k = j + 1
        else:
            k += 1
    return n, total


def analyse(run_dir, ctrl, ref, spacing):
    csvs = glob.glob(os.path.join(run_dir, 'lap_*.csv'))
    log = os.path.join(run_dir, 'controller.log')
    txt = open(log, errors='ignore').read() if os.path.exists(log) else ''
    r = {'fails': txt.count(FAIL_PATTERNS[ctrl]), 'misses': txt.count(MISS)}
    if not csvs:
        return {**r, 'moved': False}
    d = np.loadtxt(sorted(csvs)[-1], delimiter=',', skiprows=1)
    mv = np.flatnonzero(d[:, 3] > 0.2)
    if len(mv) < 2:
        return {**r, 'moved': False}
    d = d[mv[0]:mv[-1] + 1]
    t, xy, v = d[:, 0] - d[0, 0], d[:, 1:3], d[:, 3]
    dist_ref = np.linalg.norm(xy[:, None, :] - ref[None, :, :], axis=2).min(axis=1)
    clear = min(np.hypot(xy[:, 0] - ox, xy[:, 1] - oy).min() - OBS_RADIUS
                for ox, oy in OBSTACLES)
    n_stop, t_stop = stops(t, v)
    return {**r, 'moved': True, 'laps': laps_of(t, xy, ref, spacing),
            'dur': t[-1], 'v_mean': v[v > 0.1].mean() if (v > 0.1).any() else 0.0,
            'dev_max': dist_ref.max(), 'clear_min': clear,
            'n_stop': n_stop, 't_stop': t_stop, 'xy': xy}


def main():
    scen = sys.argv[1] if len(sys.argv) > 1 else 'obs'
    ref_full = np.loadtxt(f'{WS}/raceline_opt/big_0723_raceline.csv',
                          delimiter=',', skiprows=1)
    ref = ref_full[:, :2]
    spacing = float(np.median(np.linalg.norm(np.diff(ref, axis=0), axis=1)))

    rows, tracks = [], {}
    for ctrl in ('mppi', 'rpp', 'dwb'):
        for run in sorted(glob.glob(f'{WS}/laps/compare/{scen}/{ctrl}/run*')):
            a = analyse(run, ctrl, ref, spacing)
            rows.append((ctrl, os.path.basename(run), a))
            if a.get('moved') and ctrl not in tracks:
                tracks[ctrl] = a['xy']

    if not rows:
        print(f'laps/compare/{scen}/ 에 기록이 없습니다. 먼저 run_comparison.sh 를 돌리세요.')
        return

    print(f'\n=== 시나리오 {scen} ===')
    hdr = (f'{"컨트롤러":<8}{"run":<6}{"완주랩":<7}{"베스트":<8}{"평균랩":<8}'
           f'{"평균속도":<9}{"정지":<7}{"정지시간":<9}{"최대편차":<9}{"최소여유":<9}'
           f'{"실패":<7}{"주기미스":<8}')
    print(hdr)
    print('-' * len(hdr))
    for ctrl, run, a in rows:
        if not a.get('moved'):
            print(f'{ctrl:<8}{run:<6}출발 실패 (실패 {a["fails"]}회, 주기미스 {a["misses"]}회)')
            continue
        lp = a['laps']
        best = f'{min(lp):.1f}s' if lp else '-'
        mean = f'{np.mean(lp):.1f}s' if lp else '-'
        print(f'{ctrl:<8}{run:<6}{len(lp):<7}{best:<8}{mean:<8}'
              f'{a["v_mean"]:<9.2f}{a["n_stop"]:<7}{a["t_stop"]:<9.1f}'
              f'{a["dev_max"]:<9.2f}{a["clear_min"]:<9.2f}{a["fails"]:<7}{a["misses"]:<8}')

    print('\n랩=완주한 바퀴 수  베스트/평균=랩타임  정지=0.1m/s 미만 0.5초 이상 구간')
    print('최대편차=레이스라인에서 벗어난 최대 거리(m)  최소여유=장애물 표면까지 최소 거리(m)')
    print('실패=컨트롤러가 해를 못 찾은 횟수  주기미스=20Hz 제어주기 미달 횟수')

    if tracks:
        plot(scen, tracks, ref)


def plot(scen, tracks, ref):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    name = 'big_0723_obs' if scen == 'obs' else 'big_0723'
    meta = yaml.safe_load(open(f'{WS}/maps/{name}.yaml'))
    img = np.array(Image.open(f'{WS}/maps/{name}.pgm'))
    h, w = img.shape
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    fig, ax = plt.subplots(figsize=(13, 11))
    ax.imshow(img, cmap='gray', extent=[ox, ox + w * res, oy, oy + h * res], origin='upper')
    ax.plot(ref[:, 0], ref[:, 1], '--', color='0.4', lw=1, label='raceline')
    for (ctrl, xy), c in zip(tracks.items(), ('#2a78d6', '#eb6834', '#1baf7a')):
        ax.plot(xy[:, 0], xy[:, 1], '-', color=c, lw=1.5, label=ctrl.upper())
    if scen == 'obs':
        for x, y in OBSTACLES:
            ax.add_patch(plt.Circle((x, y), OBS_RADIUS, color='red', zorder=5))
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='upper right')
    ax.set_title(f'controller comparison - {scen}')
    out = f'{WS}/laps/compare/{scen}/tracks.png'
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f'\n궤적 비교 그림: {out}')


if __name__ == '__main__':
    main()
