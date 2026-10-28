#!/usr/bin/env python3
"""Independent offline validator fault checks. No ROS node or hardware startup."""
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/fast_livo2/backend'))
from validate_motion import load_pairs,matrix,read_pcd


class MotionValidation(unittest.TestCase):
    def test_nonzero_xyz_rpy_and_inverse(self):
        R=Rotation.from_euler('xyz',[.31,-.27,.83])
        pose={'xyz':[1.2,-3.4,.7],'xyzw':R.as_quat().tolist()}
        T=matrix(pose);p=np.array([.5,-.8,1.1,1.])
        np.testing.assert_allclose((T@p)[:3],R.apply(p[:3])+pose['xyz'],atol=1e-12)
        np.testing.assert_allclose(np.linalg.inv(T)@T@p,p,atol=1e-12)
        pose['xyzw'][0]=float('nan')
        with self.assertRaises(ValueError):matrix(pose)

    def test_truncated_and_nonfinite_pcd_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'map.pcd'
            path.write_bytes(b'VERSION 0.7\nFIELDS x y z intensity\n')
            with self.assertRaisesRegex(ValueError,'Truncated'):read_pcd(path)
            header=b'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 1\nDATA binary\n'
            path.write_bytes(header+np.array([1,2,3,4],dtype='<f4').tobytes())
            np.testing.assert_array_equal(read_pcd(path),[[1,2,3,4]])
            path.write_bytes(header+np.array([float('inf'),2,3,4],dtype='<f4').tobytes())
            with self.assertRaisesRegex(ValueError,'Nonfinite'):read_pcd(path)
            path.write_bytes(header+b'')
            with self.assertRaisesRegex(ValueError,'point count'):read_pcd(path)

    def test_exact_stamp_pairing_duplicate_and_missing_rejected(self):
        from rclpy.serialization import serialize_message
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);c=sqlite3.connect(folder/'native.db3')
            c.executescript('CREATE TABLE topics(id INTEGER,name TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
            c.executemany('INSERT INTO topics VALUES (?,?)',[(1,'/fast_livo/odometry'),(2,'/fast_livo/deskewed_imu')])
            o=Odometry();o.header.stamp.sec=1;o.header.stamp.nanosec=1;o.pose.pose.orientation.w=1.
            s=PointCloud2();s.header.stamp=o.header.stamp
            c.execute('INSERT INTO messages VALUES(1,1,1,?)',(serialize_message(o),))
            c.execute('INSERT INTO messages VALUES(2,2,2,?)',(serialize_message(s),));c.commit()
            self.assertEqual(load_pairs(folder)[0][0],1000000001)
            c.execute('INSERT INTO messages VALUES(3,1,3,?)',(serialize_message(o),));c.commit()
            with self.assertRaisesRegex(ValueError,'Duplicate'):load_pairs(folder)
            c.execute('DELETE FROM messages WHERE id=3');c.execute('DELETE FROM messages WHERE id=2');c.commit()
            s.header.stamp.nanosec=2
            c.execute('INSERT INTO messages VALUES(2,2,2,?)',(serialize_message(s),));c.commit()
            with self.assertRaisesRegex(ValueError,'Unpaired'):load_pairs(folder)
            c.close()


if __name__=='__main__':unittest.main(verbosity=2)
