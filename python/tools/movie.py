#! /usr/bin/env python
import os
import sys
import subprocess
import gdal
import atlaas
from atlaas.helpers.image import save
from matplotlib import cm

cmin = float(sys.argv[1])
cmax = float(sys.argv[2])
display = 'display' in sys.argv
if display:
    import matplotlib.pyplot as plt
    plt.ion()

def convert(fin, fout, cmin, cmax, cmap='viridis'):
    geo = gdal.Open(fin)
    img = geo.ReadAsArray() # get band as a numpy.array
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    if type(cmap) is str:
        if cmap in dir(cm):
            cmap = getattr(cm, cmap)
        else:
            cmap = cm.spectral
    save(fout, (cmap(img)*255).astype('uint8'))
    if display:
        imgplot = plt.imshow( img )
        imgplot.set_cmap(cmap)
        plt.draw()

pcd_id = 0
patern = 'cloud.%05i.pcd'
ftif = 'atlaas.zmean.tif'
fpng = 'atlaas.%05i.png'

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

while os.path.isfile(patern%pcd_id):
    test.merge_file(patern%pcd_id)
    test.export_zmean(ftif)
    convert(ftif, fpng%pcd_id, cmin, cmax)
    pcd_id += 1

cmd = "avconv -i atlaas.%5d.png -b 8M -an -c:v libx264 movie.avi"
print(cmd)
subprocess.check_output(cmd.split())
