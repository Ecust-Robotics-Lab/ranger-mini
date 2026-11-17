#!/usr/bin/env python3
"""Build-only experiment: preserve reviewed online source/install and vendor tree."""
import argparse,difflib,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def prepare(variant):
    source=ROOT/'ws_fast_livo/online_frontend_source'
    manifest=json.loads((source/'online_source_manifest.json').read_text())
    contents={n:(source/n).read_bytes() for n in manifest['files_sha256']}
    for n,sha in manifest['files_sha256'].items():
        if hashlib.sha256(contents[n]).hexdigest()!=sha:
            raise RuntimeError('Preserve changed online source: '+n)
    original=contents['src/LIVMapper.cpp'].decode();changed=original
    if variant=='candidate':
        before='''  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;'''
        after='''  // LIO has already synchronized this image and its IMU interval. Finish
  // that same frame's visual update without waiting for the next raw scan.
  // New LIO intervals still require all original sensor availability checks.
  const bool pending_vio = slam_mode_ == LIVO && meas.lio_vio_flg == LIO;
  if (img_buffer.empty() && img_en) return false;
  if (!pending_vio && lid_raw_data_buffer.empty() && lidar_en) return false;
  if (!pending_vio && imu_buffer.empty() && imu_en) return false;'''
        if changed.count(before)!=1:raise RuntimeError('sync availability anchor mismatch')
        changed=changed.replace(before,after)
        before='      double imu_time = stamp2Sec(imu_buffer.front()->header.stamp);'
        if changed.count(before)!=1:raise RuntimeError('unused VIO IMU read anchor mismatch')
        changed=changed.replace(before,'      // No new IMU sample is consumed by this already synchronized VIO step.')
    contents['src/LIVMapper.cpp']=changed.encode()
    replay=contents['src/ranger_livo_replay.cpp'].decode()
    before='    size_t messages=0, frames=0, vio_frames=0;'
    after='''    std::ofstream release(output + "/frame_release.csv");
    release << "sample_ns,input_receipt_ns,processing_elapsed_ms\\n";
    size_t messages=0, frames=0, vio_frames=0;'''
    if replay.count(before)!=1:raise RuntimeError('replay timing anchor mismatch')
    replay=replay.replace(before,after)
    before='        ++frames;'
    after='''        release << rclcpp::Time(odom.header.stamp).nanoseconds() << ',' << msg->time_stamp << ','
          << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count() << '\\n';
        ++frames;'''
    if replay.count(before)!=1:raise RuntimeError('replay frame anchor mismatch')
    replay=replay.replace(before,after);contents['src/ranger_livo_replay.cpp']=replay.encode()
    destination=ROOT/f'ws_fast_livo/latency_frontend_{variant}_source'
    hashes={n:hashlib.sha256(data).hexdigest() for n,data in contents.items()}
    if destination.exists():
        for n,sha in hashes.items():
            if not (destination/n).exists() or hashlib.sha256((destination/n).read_bytes()).hexdigest()!=sha:
                raise RuntimeError('Preserve changed latency source: '+n)
    else:
        for n,data in contents.items():
            p=destination/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        for n in ('pcd','image','result','Colmap/sparse/0'):(destination/'Log'/n).mkdir(parents=True,exist_ok=True)
    patch=''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),fromfile='a/src/LIVMapper.cpp',tofile='b/src/LIVMapper.cpp'))
    (destination/'latency_source_manifest.json').write_text(json.dumps(dict(variant=variant,parent_manifest=manifest,files_sha256=hashes,sync_patch=patch),indent=2))
    print(destination)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--variant',choices=('baseline','candidate'),required=True)
    prepare(p.parse_args().variant)
