import os
import os.path as osp
import numpy as np
from glob import glob
import cv2

def file_name(f):
    return osp.basename(osp.splitext(f)[0])

def read_stereo_img_list(pattern, is_concat):
    img_names = [file_name(i) for i in sorted(glob(pattern))]
    if not is_concat:
        img_names = sorted(set(i[:-2] for i in img_names))
    return img_names

def each_stereo_img(img_names, img_d, is_concat, is_vert):
    if not is_vert:
        ref_tag, tgt_tag = 'lr'
    else:
        ref_tag, tgt_tag = 'tb'

    for img_name in img_names:
        prefix = f'{img_d}/{img_name}'

        if is_concat:
            img = cv2.imread(prefix+'.jpg')
            if is_vert:
                img_ref, img_tgt = np.split(img, 2, axis=0)
            else:
                img_ref, img_tgt = np.split(img, 2, axis=1)
        else:
            img_ref = cv2.imread(prefix+f'_{ref_tag}.jpg')
            img_tgt = cv2.imread(prefix+f'_{tgt_tag}.jpg')

        yield img_name, img_ref, img_tgt, img_name+'_'+ref_tag, img_name+'_'+tgt_tag


