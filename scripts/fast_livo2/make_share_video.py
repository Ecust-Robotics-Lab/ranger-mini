#!/usr/bin/env python3
"""Create a 720p, 2x version of the demo, with its playback label updated."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if args.output.exists():parser.error('Output already exists')
    cv2.setNumThreads(2)
    reader=cv2.VideoCapture(str(args.source))
    if not reader.isOpened():raise ValueError('Unable to open source')
    width,height=int(reader.get(cv2.CAP_PROP_FRAME_WIDTH)),int(reader.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (width,height)!=(1920,1080):raise ValueError('Expected renderer 1080p layout')
    fps=reader.get(cv2.CAP_PROP_FPS)
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'avc1'),fps,(1280,720))
    if not writer.isOpened():raise ValueError('H.264 encoder unavailable')
    read=written=0
    try:
        while True:
            ok,image=reader.read()
            if not ok:break
            if read%2==0:
                # Update only the presentation HUD; no map or sensor pixels are changed.
                cv2.rectangle(image,(1640,69),(1880,112),(32,20,13),-1)
                cv2.putText(image,'2x PLAYBACK',(1670,93),cv2.FONT_HERSHEY_SIMPLEX,.55,
                            (165,184,205),1,cv2.LINE_AA)
                writer.write(cv2.resize(image,(1280,720),interpolation=cv2.INTER_AREA));written+=1
            read+=1
    finally:reader.release();writer.release()
    reader=cv2.VideoCapture(str(args.output));decoded=0
    while True:
        ok,image=reader.read()
        if not ok:break
        decoded+=1
    reader.release()
    if not written or decoded!=written or written!=(read+1)//2:raise RuntimeError('Incomplete video')
    result={'source':str(args.source.resolve()),'output':str(args.output.resolve()),'playback_speed':2,
            'resolution':[1280,720],'fps':fps,'frames':written,'decoded_frames':decoded,
            'duration_s':written/fps,'codec':'H.264 / AVC in MP4','audio':False,
            'bytes':args.output.stat().st_size,'sha256':hashlib.sha256(args.output.read_bytes()).hexdigest()}
    args.output.with_suffix('.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
