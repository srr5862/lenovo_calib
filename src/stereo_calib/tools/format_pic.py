#!/usr/bin/env python
import os
import os.path as osp
import sys

from glob import glob
import click
from shutil import copy

@click.command()
@click.argument('ts')
@click.argument('bs')
@click.argument('data')
def main(ts, bs, data):
    os.makedirs(data, exist_ok=True)
    img_ts = sorted(glob(osp.join(ts, './*.jpg')), key=lambda x:osp.basename(x)[:-4])
    img_bs = sorted(glob(osp.join(bs, './*.jpg')), key=lambda x:osp.basename(x)[:-4])
    assert len(img_ts)==len(img_bs)
    for i, (img_t, img_b) in enumerate(zip(img_ts, img_bs)):
        copy(img_t, osp.join(data, f'{i}_t.jpg'))
        copy(img_b, osp.join(data, f'{i}_b.jpg'))
        print(f"==={i}===")

if __name__ =='__main__':
    main()

