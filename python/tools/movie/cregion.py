#! /usr/bin/env python
import os
from matplotlib import cm
from atlaas.helpers.image import load, save


pcd_id = 0
patern = 'merged.region.%05i.png'

def convert(fin, fout, cmin, cmax, cmap='viridis'):
    iin = load(fin) # get band as a numpy.array
    img = iin[:,:,0]
    alp = iin[:,:,1]
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    cmap = getattr(cm, cmap)
    img = (cmap(img)*255).astype('uint8')
    img[alp < 1] = 255
    save(fout, img)

while os.path.isfile(patern%pcd_id):
    convert(patern%pcd_id, 'movie4region.%05i.png'%pcd_id, 0, 100)
    pcd_id += 1
