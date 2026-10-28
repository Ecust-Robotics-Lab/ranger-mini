#!/usr/bin/env python3
"""Offline C1 calibration. Real PNGs only; never changes drivers or source images.

Every fifth sorted image (offset 2) is held out before fitting. The exported
five-coefficient model uses training images only, keeping validation independent.
No automatic image rejection or parameter tuning against the validation set.
"""
import argparse
import csv
import hashlib
import json
import os
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

os.environ.setdefault('MPLCONFIGDIR', '/tmp/c1_matplotlib')
import cv2
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PATTERN = (11, 8)
SQUARE = 0.020
NAMES = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3']


def object_points():
    obj = np.zeros((88, 3), np.float32)
    obj[:, :2] = np.mgrid[0:11, 0:8].T.reshape(-1, 2) * SQUARE
    return obj


def detect(gray):
    ok, corners = cv2.findChessboardCornersSB(gray, PATTERN)
    if ok:
        return corners.astype(np.float32), 'SB'
    ok, corners = cv2.findChessboardCorners(
        gray, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4))
        return corners, 'traditional+cornerSubPix'
    return None, 'not detected'


def fit(records, size, flags=0):
    result = cv2.calibrateCameraExtended(
        [object_points() for _ in records], [r['corners'] for r in records],
        size, None, None, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    rms, K, D, rv, tv, std, _, per = result
    assert D.size == 5 and np.isfinite(K).all() and np.isfinite(D).all()
    return dict(rms=float(rms), K=K, D=D, rv=rv, tv=tv,
                std=std.ravel()[:9], per=per.ravel())


def metrics(residual):
    lengths = np.linalg.norm(residual.reshape(-1, 2), axis=1)
    return dict(rms=float(np.sqrt(np.mean(lengths**2))),
                mean=float(lengths.mean()), maximum=float(lengths.max()))


def project(K, D, rv, tv):
    return cv2.projectPoints(object_points(), rv, tv, K, D)[0]


def validate(records, model):
    results = []
    # Checkerboard-interleaved corners: fit pose with half; score the other half.
    grid = np.indices((8, 11))
    pose_mask = ((grid[0] + grid[1]).ravel() % 2 == 0)
    for rec in records:
        ok, rv, tv = cv2.solvePnP(object_points(), rec['corners'], model['K'], model['D'])
        if not ok:
            raise RuntimeError(f"solvePnP failed: {rec['path']}")
        residual = project(model['K'], model['D'], rv, tv) - rec['corners']
        ok, rv_half, tv_half = cv2.solvePnP(object_points()[pose_mask], rec['corners'][pose_mask],
                                          model['K'], model['D'])
        if not ok:
            raise RuntimeError('Split-corner pose fit failed')
        split_res = (project(model['K'], model['D'], rv_half, tv_half) - rec['corners'])[~pose_mask]
        results.append(dict(rec=rec, rv=rv, tv=tv, residual=residual,
                            **metrics(residual), split_rms=metrics(split_res)['rms']))
    return results


def params(model):
    K = model['K']
    return [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]),
            *map(float, model['D'].ravel())]


def write_yaml(path, data, comment=''):
    path.write_text(comment + yaml.safe_dump(data, sort_keys=False, allow_unicode=True))


def camera_info(model, size):
    K = model['K']
    P = np.column_stack([K, np.zeros(3)])
    mat = lambda a: dict(rows=int(a.shape[0]), cols=int(a.shape[1]), data=a.ravel().tolist())
    return dict(image_width=size[0], image_height=size[1], camera_name='front',
                camera_matrix=mat(K), distortion_model='plumb_bob',
                distortion_coefficients=mat(model['D'].reshape(1, 5)),
                rectification_matrix=mat(np.eye(3)), projection_matrix=mat(P))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', type=Path, required=True, help='Session directory containing images/')
    ap.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'results')
    args = ap.parse_args()
    source, out = args.input.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        raise RuntimeError('Output directory is not empty; choose a new directory to preserve results')
    (out / 'detections').mkdir()
    paths = sorted((source / 'images').glob('*.png'))
    if len(paths) < 15:
        raise RuntimeError('Need at least 15 real input PNGs for train/holdout validation')
    metadata = {}
    csv_path = source / 'capture_metadata.csv'
    if csv_path.exists():
        with csv_path.open() as f:
            metadata = {Path(r['saved_path']).name: r for r in csv.DictReader(f)
                        if r['accepted'].lower() == 'true' and r['saved_path']}
    records, invalid, size = [], [], None
    manifest = []
    cv2.setNumThreads(2)
    for image_index, path in enumerate(paths):
        if image_index % 5 == 0:
            print(f'Detecting image {image_index+1}/{len(paths)}: {path.name}', flush=True)
        im = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if im is None:
            raise RuntimeError(f'Unreadable real image: {path}')
        actual = (im.shape[1], im.shape[0])
        if size is None:
            size = actual
        if actual != size:
            raise RuntimeError(f'Mixed image resolutions: {path}: {actual} != {size}')
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        corners, method = detect(gray)
        row = metadata.get(path.name)
        wall_ns = int(row['wall_time_ns']) if row else None
        manifest.append(dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             wall_time_ns=wall_ns, header_stamp_ns=int(row['header_stamp_ns']) if row else None))
        vis = im.copy()
        if corners is None:
            invalid.append(dict(file=path.name, reason='full 11x8 corners not detected'))
            cv2.putText(vis, 'NO FULL 11x8 CORNERS', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            rec = dict(path=path, corners=corners, method=method,
                       lap=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
                       bright=float(gray.mean()), over=float((gray >= 250).mean()))
            records.append(rec)
            cv2.drawChessboardCorners(vis, PATTERN, corners, True)
            cv2.putText(vis, path.name + ' ' + method, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(str(out / 'detections' / path.name), vis)
    if len(records) < 15:
        raise RuntimeError('Insufficient complete detections')
    holdout = [r for i, r in enumerate(records) if i % 5 == 2]
    train = [r for i, r in enumerate(records) if i % 5 != 2]
    print(f'Detected {len(records)}/{len(paths)}; train={len(train)}, holdout={len(holdout)}; size={size}', flush=True)
    print('Fitting five-coefficient pinhole model...', flush=True)
    model = fit(train, size)
    validation = validate(holdout, model)
    training = []
    for rec, rv, tv in zip(train, model['rv'], model['tv']):
        residual = project(model['K'], model['D'], rv, tv) - rec['corners']
        training.append(dict(rec=rec, rv=rv, tv=tv, residual=residual, **metrics(residual), split_rms=None))
    measured_train = float(np.sqrt(np.mean([r['rms']**2 for r in training])))
    if not np.isclose(measured_train, model['rms'], atol=1e-5):
        raise RuntimeError('Independent reprojection RMS disagrees with OpenCV')
    val_rms = float(np.sqrt(np.mean([r['rms']**2 for r in validation])))
    split_rms = float(np.sqrt(np.mean([r['split_rms']**2 for r in validation])))
    print(f"Training RMS={model['rms']:.6f}; holdout RMS={val_rms:.6f}; split-corner={split_rms:.6f}", flush=True)
    # A separately fitted k3=0 model, not a silent truncation of the main model.
    print('Checking four-coefficient compatibility and three training subsets...', flush=True)
    four = fit(train, size, cv2.CALIB_FIX_K3)
    four_val = validate(holdout, four)
    four_val_rms = float(np.sqrt(np.mean([r['rms']**2 for r in four_val])))
    # Three training-only reduced-data fits reveal unstable intrinsic estimates.
    reduced = [fit([r for i, r in enumerate(train) if i % 3 != fold], size) for fold in range(3)]
    stability = np.array([params(m) for m in reduced])
    focal_change = float(np.max(np.abs(stability[:, :2] / np.array(params(model)[:2]) - 1)))
    center_change = float(np.max(np.linalg.norm(stability[:, 2:4] - np.array(params(model)[2:4]), axis=1)))
    all_corners = np.concatenate([r['corners'].reshape(-1, 2) for r in records])
    train_corners = np.concatenate([r['corners'].reshape(-1, 2) for r in train])
    hull = cv2.convexHull(all_corners.astype(np.float32))
    coverage = float(cv2.contourArea(hull) / (size[0] * size[1]))
    train_coverage = float(cv2.contourArea(cv2.convexHull(train_corners.astype(np.float32))) / (size[0] * size[1]))
    hist, _, _ = np.histogram2d(all_corners[:, 0], all_corners[:, 1], bins=(8, 6), range=((0, size[0]), (0, size[1])))
    occupied = int(np.count_nonzero(hist))
    angles = []
    for r in training + validation:
        R = cv2.Rodrigues(r['rv'])[0]
        angles.append(float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), 0, 1)))))
    # Screen for nonphysical radial folding across rays corresponding to image edges.
    pixels = np.stack(np.meshgrid(np.linspace(0, size[0]-1, 40), np.linspace(0, size[1]-1, 32)), axis=-1)
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2).astype(np.float64), model['K'], model['D'])
    r2 = np.sum(rays.reshape(-1, 2)**2, axis=1)
    k1, k2, p1, p2, k3 = model['D'].ravel()
    radial_derivative_min = float(np.min(1 + 3*k1*r2 + 5*k2*r2**2 + 7*k3*r2**3))
    errors = [r['rms'] for r in training]
    med = float(np.median(errors)); mad = float(np.median(np.abs(np.array(errors)-med)))
    suspect_threshold = max(0.5, med + 3 * 1.4826 * mad)
    suspects = [r['rec']['path'].name for r in training if r['rms'] > suspect_threshold]
    dates = [datetime.fromtimestamp(m['wall_time_ns']/1e9, ZoneInfo('Asia/Shanghai')).isoformat()
             for m in manifest if m['wall_time_ns'] is not None]
    capture_start, capture_end = (min(dates), max(dates)) if dates else ('unknown', 'unknown')
    generated = datetime.now().astimezone().isoformat()
    warnings = []
    if model['rms'] >= .5 or val_rms >= .5 or split_rms >= .5:
        warnings.append('训练或验证误差未达到 0.5 px 目标。')
    if coverage < .65 or train_coverage < .65 or occupied < 40:
        warnings.append('角点覆盖不足：边缘/角落外推的可信度有限，建议补充边缘视角。')
    if focal_change > .03 or center_change > 15:
        warnings.append('训练子集内参变化偏大，需补充姿态或检查板平整度、对焦及模型稳定性。')
    if max(angles) < 25:
        warnings.append('棋盘格倾斜幅度不足，建议增加明显倾斜视角。')
    if suspects:
        warnings.append('存在高残差候选样本，已保留并列出，未自动删除。')
    K = model['K']
    if not (K[0,0] > 0 and K[1,1] > 0 and 0 < K[0,2] < size[0] and 0 < K[1,2] < size[1] and radial_derivative_min > 0):
        warnings.append('内参合理性或径向映射单调性检查失败。')
    quality_status = 'PASS' if not warnings else 'PARTIAL'
    summary = dict(status=quality_status, generated_at=generated, capture_start=capture_start, capture_end=capture_end,
                   source=str(source), opencv_version=cv2.__version__, image_width=size[0], image_height=size[1],
                   pattern_size=list(PATTERN), square_size_m=SQUARE, camera='Hikrobot MV-CU013-A0UC',
                   lens='MVL-HF0628M-6MPE 6mm', distortion_model='plumb_bob', coefficient_order=['k1','k2','p1','p2','k3'],
                   camera_matrix=K.tolist(), distortion_coefficients=model['D'].ravel().tolist(),
                   parameters=dict(zip(NAMES, params(model))), std_deviations_intrinsics=dict(zip(NAMES, map(float,model['std']))),
                   train_rms_px=model['rms'], holdout_rms_px=val_rms, holdout_split_corner_rms_px=split_rms,
                   input_count=len(paths), train_count=len(train), holdout_count=len(holdout),
                   train_files=[r['path'].name for r in train], holdout_files=[r['path'].name for r in holdout],
                   excluded=invalid, automatically_removed_outliers=[], high_residual_candidates=suspects,
                   high_residual_threshold_px=suspect_threshold,
                   corner_hull_coverage=coverage, train_corner_hull_coverage=train_coverage, occupied_grid_cells=occupied,
                   grid_cells=48, tilt_deg_range=[min(angles),max(angles)],
                   reduced_training_focal_max_relative_change=focal_change, reduced_training_center_max_change_px=center_change,
                   radial_derivative_min=radial_derivative_min,
                   compatibility_k3_fixed_zero=dict(train_rms_px=four['rms'], holdout_rms_px=four_val_rms,
                                                    parameters=dict(zip(NAMES,params(four)))), warnings=warnings)
    write_yaml(out/'camera_intrinsics.yaml', summary)
    header = f'# Capture: {capture_start} to {capture_end}\n# Resolution: {size[0]}x{size[1]}; plumb_bob; training-only fit, independent holdout retained.\n'
    write_yaml(out/'camera_info.yaml', camera_info(model,size), header)
    fast_comment = header + (
        '# PROVISIONAL camera-only fragment: no local FAST-Calib version found.\n'
        '# Reference: https://github.com/hku-mars/FAST-Calib/blob/main/config/qr_params.yaml\n'
        '# Separate four-coefficient calibration with k3 fixed to zero; NOT the main 5-coefficient model.\n'
        '# Verify installed schema before merging. No target geometry, extrinsics, or paths are supplied.\n')
    write_yaml(out/'fast_calib_camera.yaml', dict(zip(NAMES[:8],params(four)[:8])),fast_comment)
    write_yaml(out/'compatibility_camera_info_k3_zero.yaml',camera_info(four,size),fast_comment)
    with (out/'reprojection_errors.csv').open('w',newline='') as f:
        fields=['image','split','detector','rms_px','mean_px','max_px','split_corner_validation_rms_px',
                'laplacian_var','mean_brightness','overexposed_fraction','rvec_x','rvec_y','rvec_z','tvec_x_m','tvec_y_m','tvec_z_m']
        writer=csv.writer(f); writer.writerow(fields)
        for split, rows in [('train',training),('holdout',validation)]:
            for r in rows:
                rec=r['rec']
                writer.writerow([rec['path'].name,split,rec['method'],r['rms'],r['mean'],r['maximum'],r['split_rms'],
                                 rec['lap'],rec['bright'],rec['over'],*r['rv'].ravel(),*r['tv'].ravel()])
    poses = [dict(image=r['rec']['path'].name, split=split,
                  transform='X_camera = Rodrigues(rvec) @ X_board + tvec; translation in meters',
                  rvec=r['rv'].ravel().tolist(),tvec_m=r['tv'].ravel().tolist())
             for split,rows in [('train',training),('holdout',validation)] for r in rows]
    write_yaml(out/'board_poses.yaml',dict(poses=poses))
    (out/'input_manifest.json').write_text(json.dumps(dict(images=manifest,opencv=cv2.__version__,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        split_rule='sorted successful detections: index % 5 == 2 held out; never used for intrinsic fitting',
        reduced_training_parameters=stability.tolist()),indent=2))
    # Full-resolution overlays for every input; projected points are red crosses.
    (out/'reprojections').mkdir()
    for r in training+validation:
        im=cv2.imread(str(r['rec']['path']))
        for observed, predicted in zip(r['rec']['corners'].reshape(-1,2), project(K,model['D'],r['rv'],r['tv']).reshape(-1,2)):
            cv2.circle(im,tuple(np.round(observed).astype(int)),3,(0,255,0),1)
            cv2.drawMarker(im,tuple(np.round(predicted).astype(int)),(0,0,255),cv2.MARKER_CROSS,5,1)
        cv2.imwrite(str(out/'reprojections'/r['rec']['path'].name),im)
    ordered=sorted(training+validation,key=lambda r:r['rec']['path'].name)
    fig,ax=plt.subplots(figsize=(14,4))
    ax.bar(range(len(ordered)),[r['rms'] for r in ordered],color=['#e38b2c' if r['split_rms'] is not None else '#3278b5' for r in ordered])
    ax.axhline(.5,color='red',linestyle='--',label='0.5 px target')
    ax.set_xticks(range(len(ordered)));ax.set_xticklabels([r['rec']['path'].stem.removeprefix('image_') for r in ordered],rotation=90)
    ax.set(xlabel='Image number (blue=train, orange=holdout)',ylabel='RMS reprojection error (px)');ax.legend()
    fig.tight_layout();fig.savefig(out/'reprojection_errors.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for rs,c,label in [(train,'#3278b5','train'),(holdout,'#e38b2c','holdout')]:
        points=np.concatenate([r['corners'].reshape(-1,2) for r in rs])
        axes[0].scatter(points[:,0],points[:,1],s=2,alpha=.25,color=c,label=label)
    axes[0].set(xlim=(0,size[0]),ylim=(size[1],0),aspect='equal',title=f'Corner coverage; hull {coverage:.1%}')
    axes[0].legend();heat=axes[1].imshow(hist.T,extent=(0,size[0],size[1],0),aspect='equal',cmap='viridis')
    axes[1].set_title(f'8x6 grid: {occupied}/48 cells occupied');fig.colorbar(heat,ax=axes[1],label='Corner observations')
    fig.tight_layout();fig.savefig(out/'corner_coverage.png',dpi=160);plt.close(fig)
    # Three holdout views, full sensor dimensions retained; no ROI crop or resize.
    compare=sorted(validation,key=lambda r:r['rec']['path'].name)
    picks=[compare[0],compare[len(compare)//2],compare[-1]]
    (out/'undistorted').mkdir()
    newK,roi=cv2.getOptimalNewCameraMatrix(K,model['D'],size,1,size)
    write_yaml(out/'undistortion_camera_info.yaml',camera_info(dict(K=newK,D=np.zeros((1,5))),size),
               '# For alpha=1 undistorted images only; do not use with raw images.\n')
    fig,axes=plt.subplots(3,2,figsize=(12,15))
    for row,r in enumerate(picks):
        raw=cv2.imread(str(r['rec']['path'])); und=cv2.undistort(raw,K,model['D'],None,newK)
        cv2.imwrite(str(out/'undistorted'/r['rec']['path'].name),und)
        for col,im in enumerate([raw,und]):
            axes[row,col].imshow(cv2.cvtColor(im,cv2.COLOR_BGR2RGB));axes[row,col].axis('off')
            axes[row,col].set_title(r['rec']['path'].name+(' raw' if col==0 else ' undistorted alpha=1'))
    fig.tight_layout();fig.savefig(out/'undistortion_comparison.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,5))
    residuals=np.concatenate([r['residual'].reshape(-1,2) for r in ordered])
    ax.scatter(residuals[:,0],residuals[:,1],s=3,alpha=.2);ax.axhline(0,color='gray');ax.axvline(0,color='gray')
    ax.set(xlabel='dx (px)',ylabel='dy (px)',title='All residual vectors (predicted - detected)',aspect='equal')
    fig.tight_layout();fig.savefig(out/'residual_scatter.png',dpi=150);plt.close(fig)
    parameter_table='\n'.join(f'| {name} | {value:.10g} | {float(std):.5g} |' for name,value,std in zip(NAMES,params(model),model['std']))
    per_table='\n'.join(f"| {r['rec']['path'].name} | {'验证' if r['split_rms'] is not None else '训练'} | {r['rms']:.4f} | {r['maximum']:.4f} |" for r in ordered)
    warning_text='\n'.join('- '+w for w in warnings) if warnings else '- 本轮数值检查未发现超过所列阈值的异常；仍不等同于全视场绝对几何精度认证。'
    report=f'''# 阶段 C1：海康工业相机内参标定报告

结论：**{quality_status}**（适用范围：本批静态图片上的针孔模型拟合与离线验证）。FAST-Calib 实际版本适配为 **PARTIAL / 待核实**，未部署任何配置。

## 数据与方法

- 相机：Hikrobot MV-CU013-A0UC；镜头 MVL-HF0628M-6MPE，6 mm。曝光 20000 μs、增益 10 为用户确认的采集设置，PNG 本身不包含该 SDK 状态。
- 数据：`{source}`；实际分辨率 **{size[0]}×{size[1]}**，全程未缩放、裁剪原图。
- 实际采集时间：{capture_start} 至 {capture_end}（Asia/Shanghai，来自 metadata 的本机 wall_time_ns）。生成：{generated}。
- header_stamp_ns 属 Stage B 雷达时间域，不能作为民用日期；原始时间戳和 SHA256 保存在 input_manifest.json。
- 棋盘格 12×9 方格，11×8 内角点，边长 0.020 m；物点 (x,y,0)，x=0..10、y=0..7，间距 0.020 m。
- 输入 {len(paths)} 张；完整检测 {len(records)} 张；训练 {len(train)} 张，固定保留验证 {len(holdout)} 张。先按文件名排序，索引 %5==2 为验证，其余训练。
- OpenCV {cv2.__version__}：优先 findChessboardCornersSB，失败才采用传统检测+cornerSubPix；calibrateCameraExtended，flags=0，标准五参数 plumb_bob，未开启鱼眼或高阶模型，未输入理论焦距或占位内参。
- **导出的主模型只用训练图片求解。验证图片从未参与主模型内参拟合，未在验证后用全量重拟合。** 验证图仅估计各自外姿态。半角点验证用交错一半角点估计姿态，另一半评分。
- 检测失败排除：{json.dumps(invalid,ensure_ascii=False)}。未按误差自动剔除任何图片。训练高残差提示阈值 max(0.5, median+3×1.4826×MAD)={suspect_threshold:.4f} px；候选：{', '.join(suspects) or '无'}。

## 主模型结果

| 参数 | 估计值 | OpenCV 近似标准差 |
|---|---:|---:|
{parameter_table}

焦距和主点单位为 px；畸变无量纲。标准差为当前模型的局部近似，不包含标定板误差、测量误差、焦点漂移等系统误差。

```text
K = {np.array2string(K,precision=9)}
D = {np.array2string(model['D'].ravel(),precision=9)}
```

- 训练整体 RMS：**{model['rms']:.6f} px**，已用 projectPoints 独立重算核对。
- 保留图像整体 RMS：**{val_rms:.6f} px**。
- 保留图像半角点姿态拟合、另一半评分 RMS：**{split_rms:.6f} px**。
- RMS 定义 sqrt(mean(dx²+dy²))，不是每坐标维度 RMS。每张均为 88 个角点。
- 每图旋转向量和平移见 board_poses.yaml、reprojection_errors.csv。变换为 X_camera=Rodrigues(rvec)×X_board+tvec，平移单位米；这些是棋盘格外姿态，**不是雷达到相机外参**。

## 质量、覆盖与稳定性

- 全部角点凸包覆盖成像平面 {coverage:.2%}；训练角点凸包覆盖 {train_coverage:.2%}。凸包表示观测区域边界，并不代表内部均匀密集覆盖。
- 8×6 网格占用 {occupied}/48；详见 corner_coverage.png。未覆盖的画面边缘不应视为已验证区域。
- 棋盘法向与光轴夹角 {min(angles):.2f}°–{max(angles):.2f}°；这是由拟合姿态估计的倾斜幅度。
- 三次训练内部删去不同三分之一视图再拟合，焦距最大相对变化 {focal_change:.2%}，主点最大移动 {center_change:.3f} px。该检查未使用验证集选择模型。
- fx/fy={K[0,0]/K[1,1]:.6f}；主点相对画面中心偏移 ({K[0,2]-size[0]/2:.3f}, {K[1,2]-size[1]/2:.3f}) px。
- 成像平面采样射线上的径向映射导数最小值 {radial_derivative_min:.6f}；正值只排除了所检查范围内的径向折叠，并非完整的畸变可逆性证明。
- 本报告预设提示阈值：训练/验证 RMS≥0.5 px；凸包覆盖<65%或网格占用<40/48；子集焦距变化>3%或主点变化>15 px；最大倾斜<25°。这些是工程检查阈值，不是制造商精度保证。

{warning_text}

板平整度由用户确认，图像数据无法单独证明；纸面印刷精度、失焦或运动模糊会影响系统误差。全图拉普拉斯方差和亮度仅作辅助，不构成棋盘局部清晰度认证。若边缘误差偏高，应优先补充边缘/倾斜视图并检查反光、板平整度与镜头固定情况，不能仅通过删图压低 RMS。

![角点覆盖](corner_coverage.png)
![逐图误差](reprojection_errors.png)
![去畸变对比](undistortion_comparison.png)

## FAST-Calib 兼容性

本工程已整合支持完整 k3 的 FAST-Calib 适配。此脚本导出的 fast_calib_camera.yaml 仍是旧四参数接口的兼容片段，不能覆盖主五参数模型；正式外参流程通过本工程 make-config 生成完整五参数配置。

fast_calib_camera.yaml 是**暂存的相机参数片段**，使用同一训练集单独约束 k3=0 重算，避免把五参数模型直接截断：训练 RMS {four['rms']:.6f} px，保留图像 RMS {four_val_rms:.6f} px。主模型仍为五参数模型。对应 CameraInfo 另存 compatibility_camera_info_k3_zero.yaml，不能混用两组内参。未填写或更改标定目标几何、雷达话题、外参或任何现有配置。使用前必须核对实际安装的 FAST-Calib 版本及其参数加载代码。

## 文件与复现

- camera_intrinsics.yaml：主模型、标准差、划分、质量指标、排除记录和采集日期。
- camera_info.yaml：ROS camera_info_manager 兼容格式，原始 {size[0]}×{size[1]}，D 顺序 k1,k2,p1,p2,k3。P=[K|0]，R=I；未装载到驱动。
- detections/：每张原图角点可视化；reprojections/：观测绿圈、投影红叉。
- reprojection_errors.csv / .png：逐图误差、质量指标、姿态。
- corner_coverage.png、residual_scatter.png：覆盖与残差。
- undistortion_comparison.png：三张保留图片原图/去畸变对比，仅展示图拼版缩放；undistorted/ 中 PNG 保持原分辨率、不裁剪。
- undistortion_camera_info.yaml：去畸变图像对应的新 K（alpha=1）、零 D；不能用于原始图像。黑边为保留全视场的正常结果。
- board_poses.yaml、input_manifest.json：每图姿态、图像 SHA256、程序 SHA256、固定划分和训练子集参数。

```bash
cd {Path(__file__).resolve().parent.parent}
python3 camera_intrinsics/calibrate_intrinsics.py \\
  --input {source} \\
  --output {Path(__file__).resolve().parent}/results_repeat_$(date +%Y%m%d_%H%M%S)
```

本次执行正常完成，未修改源图片、Stage B 同步源码、ranger_nav 或 FAST-LIVO2 配置。目录非空时拒绝覆盖。算法参考 [OpenCV calib3d 官方文档](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html)。

## 逐图误差

| 图片 | 用途 | RMS (px) | 最大角点误差 (px) |
|---|---|---:|---:|
{per_table}
'''
    (out/'calibration_report.md').write_text(report)
    print(json.dumps({k:summary[k] for k in ['status','parameters','train_rms_px','holdout_rms_px','holdout_split_corner_rms_px','corner_hull_coverage','warnings']},ensure_ascii=False,indent=2),flush=True)
    print('Results:',out,flush=True)


if __name__ == '__main__':
    main()
