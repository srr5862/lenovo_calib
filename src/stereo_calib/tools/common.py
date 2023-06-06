import os
import os.path as osp
import numpy as np
import cv2
from pycbdetect import get_chessboard_3d_points, detect_chessboard

def file_name(f):
    return osp.basename(osp.splitext(f)[0])

def camo_get_K(c):
    p = c.getParameters()
    return np.array([
        [p.m_mu, 0, p.m_u0],
        [0, p.m_mv, p.m_v0],
        [0, 0, 1],
    ])

def camo_get_dist(c):
    p = c.getParameters()
    return [p.m_k2, p.m_k3, p.m_k4, p.m_k5]

def stereo_init_undistort_map(cfg):
    fn = cv2.fisheye.initUndistortRectifyMap if cfg['model'] == 'ocv_fisheye' else cv2.initUndistortRectifyMap
    img_size = tuple(cfg['img_size'])

    K, D = [np.asarray(cfg['ref']['ori'][k]) for k in ['K', 'D']]
    R, P = [np.asarray(cfg['ref']['rectified'][k]) for k in ['R', 'P']]
    map_ref = cv2.initUndistortRectifyMap(K, D, R, P, img_size, cv2.CV_16SC2)

    K, D = [np.asarray(cfg['tgt']['ori'][k]) for k in ['K', 'D']]
    R, P = [np.asarray(cfg['tgt']['rectified'][k]) for k in ['R', 'P']]
    map_tgt = cv2.initUndistortRectifyMap(K, D, R, P, img_size, cv2.CV_16SC2)
    return map_ref, map_tgt

class StereoRemapper:
    def __init__(self, m):
        if isinstance(m, dict):
            m = stereo_init_undistort_map(m)

        self.map_ref, self.map_tgt = m

    def remap(self, img_ref, img_tgt, interp=cv2.INTER_LINEAR, **kws):
        img_ref1 = cv2.remap(img_ref, *self.map_ref, interp, **kws)
        img_tgt1 = cv2.remap(img_tgt, *self.map_tgt, interp, **kws)
        return img_ref1, img_tgt1
