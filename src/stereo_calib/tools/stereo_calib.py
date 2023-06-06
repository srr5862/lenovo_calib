#!/usr/bin/env python
import os
import os.path as osp
import numpy as np
import numpy.linalg as npl
import cv2
import toml
import yaml
from glob import glob
import re
from pycbdetect import detect_chessboard
from common import *
from io_ import *
import pylab as plt
import click
from easydict import EasyDict as edict


@click.command()
@click.option('--cfg', default='config/stereo_config.toml')
@click.option('--data', default='data')
def main(cfg, data):
    with open(cfg) as f:
        cfg = edict(toml.load(f)).config
    img_d = data
    out_f = cfg.out_f
    cb_d = cfg.cb_d
    rectified_d = cfg.rectified_d

    cb_size = tuple(cfg.cb_size)
    cb_w, cb_h = cb_size

    objp = get_chessboard_3d_points(cb_size, cfg.square_size)
    objp = objp.reshape((cb_h*cb_w, 1, 3))

    if not cfg.is_vert:
        ref_tag, tgt_tag = 'lr'
    else:
        ref_tag, tgt_tag = 'tb'

    img_names = read_stereo_img_list(img_d+'/*.jpg', cfg.is_concat)
    if cb_d:
        os.makedirs(cb_d, exist_ok=True)
    if rectified_d:
        os.makedirs(rectified_d, exist_ok=True)

    img_pts_ref, img_pts_tgt = [], []
    obj_pts = []
    imh, imw = None, None

    imgs = []

    for img_name, img_ref, img_tgt, ref_name, tgt_name in each_stereo_img(img_names, img_d, cfg.is_concat, cfg.is_vert):
        prefix = img_d + '/' + img_name
        imgs.append((img_ref, img_tgt))

        print (f"detecting {ref_name}")
        if imh is None:
            imh, imw = img_ref.shape[:2]

        corners_ref = detect_chessboard(img_ref, cb_size, use_opencv=cfg.ocv_det_cb)
        if corners_ref is None:
            print (f"{prefix} ref cant detect!")
            continue

        corners_ref = corners_ref.reshape(-1, 2)
        if cb_d:
            canvas = img_ref.copy()
            canvas = cv2.drawChessboardCorners(img_ref, cb_size, corners_ref, True)
            cv2.imwrite(cb_d+'/'+ref_name+'.jpg', canvas)

        print (f"detecting {tgt_name}")
        corners_tgt = detect_chessboard(img_tgt, cb_size, use_opencv=cfg.ocv_det_cb)
        if corners_tgt is None:
            print (f"{prefix} tgt cant detect!")
            continue

        corners_tgt = corners_tgt.reshape(-1, 2)
        if cb_d:
            canvas = img_tgt.copy()
            canvas = cv2.drawChessboardCorners(img_tgt, cb_size, corners_tgt, True)
            cv2.imwrite(cb_d+'/'+tgt_name+'.jpg', canvas)

        img_pts_ref.append(corners_ref)
        img_pts_tgt.append(corners_tgt)

        obj_pts.append(objp)

    img_size = (imw, imh)

    N = len(obj_pts)
    dtype = 'f8' if cfg.is_fisheye else 'f4'
    obj_pts = np.asarray(obj_pts, dtype)
    obj_pts = np.reshape(obj_pts, (N, 1, -1, 3))
    img_pts_ref = np.asarray(img_pts_ref, dtype)
    img_pts_ref = np.reshape(img_pts_ref, (N, 1, -1, 2))
    img_pts_tgt = np.asarray(img_pts_tgt, dtype)
    img_pts_tgt = np.reshape(img_pts_tgt, (N, 1, -1, 2))

    if cfg.is_fisheye:
        calib_fn = cv2.fisheye.calibrate
        stereo_calib_fn = cv2.fisheye.stereoCalibrate
        stereo_rectify_fn = cv2.fisheye.stereoRectify
        init_undistort_map_fn = cv2.fisheye.initUndistortRectifyMap
        calib_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW
        stereo_flags = cv2.CALIB_FIX_INTRINSIC
    else:
        calib_fn = cv2.calibrateCamera
        stereo_calib_fn = cv2.stereoCalibrate
        stereo_rectify_fn = cv2.stereoRectify
        init_undistort_map_fn = cv2.initUndistortRectifyMap
        calib_flags = 0
        stereo_flags = cv2.CALIB_FIX_INTRINSIC

    rms_ref, K_ref, D_ref, rvecs_ref, tvec_ref = calib_fn(obj_pts, img_pts_ref, img_size, None, None, flags=calib_flags)
    #K_ref, roi_ref = cv2.getOptimalNewCameraMatrix(K_ref, D_ref, (imw, imh), alpha=cfg.alpha, newImgSize=(imw, imh))

    rms_tgt, K_tgt, D_tgt, rvecs_tgt, tvec_tgt = calib_fn(obj_pts, img_pts_tgt, img_size, None, None, flags=calib_flags)
    #K_tgt, roi_tgt = cv2.getOptimalNewCameraMatrix(K_tgt, D_tgt, (imw, imh), alpha=cfg.alpha, newImgSize=(imw, imh))

    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    rms, K_ref, D_ref, K_tgt, D_tgt, R, T = stereo_calib_fn(obj_pts, img_pts_ref, img_pts_tgt, K_ref, D_ref, K_tgt, D_tgt, img_size, crit, stereo_flags)[:7]

    print (f"rms: {rms}, {rms_ref}, {rms_tgt}")
    R_ref, R_tgt, P_ref, P_tgt, Q = stereo_rectify_fn(K_ref, D_ref, K_tgt, D_tgt, img_size, R, T, cfg.alpha)[:5]

    for i in range(3):
        if R_ref[i,i] < 0 and R_tgt[i,i] < 0:
            R_ref[i,:] *= -1
            R_tgt[i,:] *= -1

    if rectified_d:
        map_ref = init_undistort_map_fn(K_ref, D_ref, R_ref, P_ref, img_size, cv2.CV_16SC2)
        map_tgt = init_undistort_map_fn(K_tgt, D_tgt, R_tgt, P_tgt, img_size, cv2.CV_16SC2)
        remapper = StereoRemapper((map_ref, map_tgt))
        for img_name, (img_ref, img_tgt) in zip(img_names, imgs):
            img_pairs = remapper.remap(img_ref, img_tgt)
            if cfg.is_vert:
                canvas = np.vstack(img_pairs)
            else:
                canvas = np.hstack(img_pairs)

            cv2.imwrite(rectified_d+f'/{img_name}.jpg', canvas)
    out = {
        'model': ('ocv_fisheye' if cfg.is_fisheye else 'ocv'),
        'is_vert': cfg.is_vert,
        'ref': {
            'ori': {
                'K': K_ref.tolist(),
                'D': D_ref.tolist(),
            },
            'rectified':{
                'R': R_ref.tolist(),
                'P': P_ref.tolist(),
                'K': P_ref[:3, :3].tolist(),
            },
        },
        'tgt': {
            'ori':{
                'K': K_tgt.tolist(),
                'D': D_tgt.tolist(),
            },
            'rectified':{
                'R': R_tgt.tolist(),
                'P': P_tgt.tolist(),
                'K': P_tgt[:3, :3].tolist(),
            }
        },
        'img_size': list(img_size),
        'ori_R': R.tolist(),
        'ori_T': T.tolist(),
        'rectified_Q': Q.tolist(),
        'rectified_baseline': float(1/Q[3, 2]),
        'rms': rms,
        'rms_ref': rms_ref,
        'rms_tgt': rms_tgt,
    }
    yaml.dump(out, open(out_f, 'w'))

if __name__ == '__main__':
    main()
