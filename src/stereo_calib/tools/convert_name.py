import os
import os.path as osp
import sys
from glob import glob

root = sys.argv[1]

imgs = glob(f"{root}/*.jpg")

for img in imgs:
    num = int(osp.basename(img)[:-4])
    if num % 2 == 0:
        os.rename(img, osp.join(osp.dirname(img), f'{int(num/2)}_b.jpg'))
    else:
        os.rename(img, osp.join(osp.dirname(img), f'{int(num/2)+1}_t.jpg'))


