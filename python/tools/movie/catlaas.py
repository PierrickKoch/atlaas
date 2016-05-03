#! /usr/bin/env python
import os
from matplotlib import cm
from atlaas.helpers.gdal2 import gdal2
from atlaas.helpers.image import save

pcd_id = 0
patern = 'merged.atlaas.%05i.tif'

def convert(fin, fout, cmin, cmax, cmap='viridis'):
    geo = gdal2(fin)
    npt = geo.bands[geo.names["N_POINTS"]]
    img = geo.bands[geo.names["Z_MEAN"]]
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    cmap = getattr(cm, cmap)
    img = (cmap(img)*255).astype('uint8')
    img[npt < 1] = 0
    save(fout, img)

while os.path.isfile(patern%pcd_id):
    convert(patern%pcd_id, 'movie4atlaas.%05i.png'%pcd_id, 2, 5)
    pcd_id += 1
