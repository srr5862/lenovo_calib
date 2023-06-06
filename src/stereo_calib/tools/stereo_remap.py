#!/usr/bin/env python
import os
import os.path as osp
import numpy as np
import yaml
import cv2
from glob import glob
from common import StereoRemapper
from io_ import *
import click
from tqdm import tqdm

def file_name(f):
    return osp.splitext(osp.basename(f))[0]

@click.command()
@click.option('--img-d', default='data')
@click.option('--out-d', default='rectified_images')
@click.option('--config', default='stereo_calib.yaml')
@click.option('--is-concat', is_flag=True, default=False)
@click.option('--is-vert', is_flag=True, default=True)
def main(img_d, out_d, config, is_concat, is_vert):
    with open(config) as f:
        cfg = yaml.load(f, Loader=yaml.SafeLoader)
    remapper = StereoRemapper(cfg)

    img_names = read_stereo_img_list(img_d+'/*.jpg', is_concat)

    img_names = [file_name(i) for i in sorted(glob(img_d+'/*.jpg'))]
    if not is_concat:
        img_names = sorted(set(i[:-2] for i in img_names))

    os.makedirs(out_d, exist_ok=True)

    for img_name, img_ref, img_tgt, ref_name, tgt_name in tqdm(each_stereo_img(img_names, img_d, is_concat, is_vert), total=len(img_names)):
        img_ref, img_tgt = remapper.remap(img_ref, img_tgt)
        cv2.imwrite(out_d+'/'+ref_name+'.jpg', img_ref)
        cv2.imwrite(out_d+'/'+tgt_name+'.jpg', img_tgt)

if __name__ == '__main__':
    main()

