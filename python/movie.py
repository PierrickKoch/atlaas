#! /usr/bin/env python
import gdal
import Image
import atlaas
import subprocess
import numpy as np
from matplotlib import cm, pyplot

cmin = -1
cmax = 10
pcds = 533
folder = '2014_08_20_16_41_18_velodyne_pcd2'

#pyplot.ion()
def convert(fin, fout, cmin, cmax, cmap=cm.spectral):
    geo = gdal.Open(fin)
    img = geo.ReadAsArray() # get band as a numpy.array
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    # in case of JPEG or WebP, set quality to 90%, else this option is ignored
    Image.fromarray(np.uint8(cmap(img)*255)).save(fout, quality=90)
    #imgplot = pyplot.imshow( img )
    #imgplot.set_cmap(cmap)
    #pyplot.draw()

patern = folder+'/cloud.%05i.pcd'
ftif = 'atlaas.zmean.tif'
fpng = 'atlaas.%05i.png'

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

for pcd_id in range(pcds):
    test.merge_file(patern%pcd_id)
    test.export_zmean(ftif)
    convert(ftif, fpng%pcd_id, cmin, cmax)

cmd = "avconv -i atlaas.%5d.png -b 8M -an -c:v libx264 movie.avi"
print(cmd)
subprocess.check_output(cmd.split())
