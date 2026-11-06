#!/usr/bin/env python3
"""Focus/aperture adjustment viewer for MV-CU013 (replaces crashing MVS GUI).

SDK free-run capture (TriggerMode Off, volatile settings) + OpenCV window:
  - live image + zoomed center ROI
  - numeric focus metric: Laplacian variance (higher = sharper)
  - mean brightness
Keys:
  +/-   exposure +/- 2000 us        q/ESC  quit
  g/G   gain +/- 2                  (all camera settings are volatile)
"""
import os
import sys, time
os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
sys.path.append("/opt/MVS/Samples/64/Python")
from MvImport.MvCameraControl_class import *
from MvImport.CameraParams_header import MV_USB_DEVICE, MV_GIGE_DEVICE, MV_TRIGGER_MODE_OFF
from ctypes import *
import cv2
import numpy as np

SN = "DB1216907"
EXPO0, GAIN0 = 20000.0, 10.0

dl = MV_CC_DEVICE_INFO_LIST()
assert MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, dl) == 0
dev = next((cast(dl.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            for i in range(dl.nDeviceNum)
            if cast(dl.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents.nTLayerType == MV_USB_DEVICE
            and bytes(cast(dl.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
                      .SpecialInfo.stUsb3VInfo.chSerialNumber).split(b"\x00")[0].decode() == SN),
           None)
assert dev is not None, f"camera {SN} not found"

cam = MvCamera()
assert cam.MV_CC_CreateHandle(dev) == 0
assert cam.MV_CC_OpenDevice() == 0, "open failed (busy?)"

cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
cam.MV_CC_SetEnumValue("ExposureAuto", 0)
cam.MV_CC_SetFloatValue("ExposureTime", EXPO0)
cam.MV_CC_SetEnumValue("GainAuto", 0)
cam.MV_CC_SetFloatValue("Gain", GAIN0)
cam.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", False)

st = MVCC_INTVALUE()
cam.MV_CC_GetIntValue("PayloadSize", st)
buf = (c_ubyte * (st.nCurValue * 3 + 4096))()
info = MV_FRAME_OUT_INFO_EX()
assert cam.MV_CC_StartGrabbing() == 0

expo, gain = EXPO0, GAIN0
print("focus viewer: +/- exposure, g/G gain, q quit")

def sharp(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    roi = g[h//4:3*h//4, w//4:3*w//4]
    return cv2.Laplician(roi, cv2.CV_64F).var() if False else cv2.Laplacian(roi, cv2.CV_64F).var(), float(g.mean())

best = 0.0
while True:
    ret = cam.MV_CC_GetOneFrameTimeout(buf, sizeof(buf), info, 800)
    if ret != 0:
        print("frame timeout")
        continue
    img = np.frombuffer(buf, dtype=np.uint8, count=info.nWidth*info.nHeight*3
                        if info.enPixelType & 0x00800000 else info.nWidth*info.nHeight) \
        if False else None
    # USB Bayer8: nFrameLen bytes grayscale -> debayer
    n = info.nWidth * info.nHeight
    raw = np.frombuffer(buf, dtype=np.uint8, count=n).reshape(info.nHeight, info.nWidth)
    bgr = cv2.cvtColor(raw, cv2.COLOR_BayerGB2BGR)
    lap, mean = sharp(bgr)
    best = max(best, lap)
    h, w = bgr.shape[:2]
    roi = bgr[h//4:3*h//4, w//4:3*w//4]
    zoom = cv2.resize(roi, (w//2, h//2), interpolation=cv2.INTER_NEAREST)
    zoom = cv2.rectangle(zoom, (5, 5), (zoom.shape[1]-5, zoom.shape[0]-5), (0,255,255), 1)
    bgr[:zoom.shape[0], w-zoom.shape[1]:] = zoom
    cv2.putText(bgr, f"SHARP(ROI lap var): {lap:8.0f}   best: {best:8.0f}", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0) if lap >= best-1 else (0,200,255), 2)
    cv2.putText(bgr, f"bright {mean:5.1f}  expo {expo/1000:.0f}ms  gain {gain:.1f}", (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 1)
    cv2.imshow("focus", bgr)
    k = cv2.waitKey(20) & 0xFF
    if k in (ord('q'), 27):
        break
    if k in (ord('+'), ord('=')) and expo < 90000:
        expo += 2000; cam.MV_CC_SetFloatValue("ExposureTime", expo)
    if k in (ord('-'), ord('_')) and expo > 2000:
        expo -= 2000; cam.MV_CC_SetFloatValue("ExposureTime", expo)
    if k == ord('g') and gain < 17:
        gain += 2; cam.MV_CC_SetFloatValue("Gain", gain)
    if k == ord('G') and gain > 0:
        gain -= 2; cam.MV_CC_SetFloatValue("Gain", gain)

cam.MV_CC_StopGrabbing()
cam.MV_CC_CloseDevice()
cam.MV_CC_DestroyHandle()
cv2.destroyAllWindows()
print(f"final best sharpness: {best:.0f} (settings revert on close)")
