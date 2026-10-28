"""Numerical/interface tests; these are NOT real-board calibration evidence."""
import sys, unittest, tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
import cv2
from sensor_msgs.msg import PointCloud2,PointField,Image
from array import array
class Interfaces(unittest.TestCase):
    def test_real_k3_projection_and_inverse(self):
        intr=intrinsics(ROOT.parent/'camera_intrinsics/results/camera_intrinsics.yaml');K=np.array(intr['K']);D=np.array(intr['D']);x,y=np.meshgrid(np.linspace(-.45,.45,13),np.linspace(-.35,.35,11));x=x.ravel();y=y.ravel();p=np.c_[x,y,np.ones(len(x))];r2=x*x+y*y;k1,k2,p1,p2,k3=D;rad=1+k1*r2+k2*r2**2+k3*r2**3
        expected=np.c_[(x*rad+2*p1*x*y+p2*(r2+2*x*x))*K[0,0]+K[0,2],(y*rad+p1*(r2+2*y*y)+2*p2*x*y)*K[1,1]+K[1,2]]
        uv=cv2.projectPoints(p,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2);err=float(abs(uv-expected).max());self.assertLess(err,1e-9)
        und=cv2.undistortPointsIter(uv.reshape(-1,1,2),K,D,None,K,(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,50,1e-13)).reshape(-1,2);target=np.c_[x*K[0,0]+K[0,2],y*K[1,1]+K[1,2]];inv=float(abs(und-target).max());self.assertLess(inv,1e-8)
        d0=D.copy();d0[4]=0;delta=float(np.linalg.norm(cv2.projectPoints(p,np.zeros(3),np.zeros(3),K,d0)[0].reshape(-1,2)-uv,axis=1).max());print('REAL_K3_NUMERICS: projection max',err,'px; undistort max',inv,'px; dropping k3 max',delta,'px')
    def test_mid360s_layout_and_padding(self):
        import struct
        m=PointCloud2();m.height=2;m.width=2;m.point_step=26;m.row_step=56
        m.fields=[PointField(name=k,offset=o,datatype=7,count=1) for k,o in [('x',0),('y',4),('z',8)]]
        data=bytearray(112)
        for row in range(2):
            for col in range(2):struct.pack_into('<fff',data,row*56+col*26,1+row,2+col,3)
        m.data=array('B',data);self.assertEqual(xyz(m).shape,(4,3));np.testing.assert_allclose(xyz(m)[-1],[2,3,3])
    def test_rgb_bgr(self):
        for encoding,bytes_ in [('rgb8',[10,20,30]),('bgr8',[30,20,10])]:
            m=Image();m.width=m.height=1;m.step=3;m.encoding=encoding;m.data=array('B',bytes_);np.testing.assert_equal(bgr(m)[0,0],[30,20,10])
    def test_unverified_board_orientation_rejected(self):
        # Field verification updates the real board config; test an isolated invalid fixture.
        with tempfile.TemporaryDirectory(dir=ROOT/'logs') as temp:
            fixture=Path(temp)/'board.yaml';data=read(ROOT/'config/board.yaml');data['orientation_verified']=False
            fixture.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):board(fixture)
            data['orientation_verified']=True;data['marker_size']='UNMEASURED';fixture.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):board(fixture)
    def test_paired_projection_skips_unmatched_first_image(self):
        from unittest.mock import patch
        import struct
        def image(ns,value):
            m=Image();m.width=m.height=1;m.step=3;m.encoding='bgr8';m.header.frame_id='optical';m.header.stamp.nanosec=ns;m.data=array('B',[value]*3);return m
        cloud=PointCloud2();cloud.header.frame_id='livox';cloud.header.stamp.nanosec=11;cloud.width=cloud.height=1;cloud.point_step=cloud.row_step=12
        cloud.fields=[PointField(name=k,offset=o,datatype=7,count=1) for k,o in [('x',0),('y',4),('z',8)]];cloud.data=array('B',struct.pack('<fff',1,2,3))
        sequence=[(IMAGE,image(10,10),0),(LIDAR,cloud,1),(IMAGE,image(11,11),2)]
        with patch('common.messages',return_value=iter(sequence)):
            im,points,info=paired_sample('in-memory-fixture',1)
            self.assertEqual(info['image_stamp_ns'],11);self.assertEqual(info['lidar_first_stamp_ns'],11)
            self.assertEqual(info['skipped_unpaired_images_before_first_pair'],1);self.assertEqual(int(im[0,0,0]),11)
        with patch('common.messages',return_value=iter(sequence)):
            with self.assertRaises(ValueError):paired_sample('in-memory-fixture',2)
    def test_ros_parameter_yaml_no_aliases(self):
        doc=ros_parameter_document({'fx':1300.0,'configuration_validated':True,'max_frames':50})
        text=yaml.safe_dump(doc)
        self.assertNotIn('&id',text);self.assertNotIn('*id',text)
        self.assertEqual(yaml.safe_load(text),doc)
    def test_bad_rotation_rejected(self):
        t=np.eye(4);t[0,0]=-1
        with self.assertRaises(ValueError):matrix_check(t)
    def test_inverse(self):
        t=np.eye(4);t[:3,:3]=cv2.Rodrigues(np.array([.1,-.2,.3]))[0];t[:3,3]=[.3,.1,.2];self.assertLess(matrix_check(t)['inverse_error'],1e-12)
if __name__=='__main__':unittest.main(verbosity=2)
