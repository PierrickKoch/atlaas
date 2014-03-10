filename = 'out.tif'

import gdal
g = gdal.Open( filename )
names = {v:k+1 for k,v in enumerate(["N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "VARIANCE", "TIME"])}
'''
b1 = g.GetRasterBand(names['N_POINTS'])
r1 = b1.ReadAsArray()
b5 = g.GetRasterBand(names['VARIANCE'])
r5 = b5.ReadAsArray()

f1 = r1.flatten()
f5 = r5.flatten()
t = 0
c = 0
# todo numpy must have smart way of doing this
for n,v in zip(f1, f5):
    if n > 1:
        t += v
        c += 1

m = t / c
m * 3.
'''
# import numpy as np
# import matplotlib.mlab as mlab
# or run `ipython --pylab`
import matplotlib.pyplot as plt
'''
f4 = r4.flatten()
plt.hist( f4, 256 )
plt.show()
'''
g = gdal.Open( filename )
b4 = g.GetRasterBand(names['Z_MEAN'])
r4 = b4.ReadAsArray()

imgplot = plt.imshow( r4 )
imgplot.set_clim(-1.0, 3.0)
plt.colorbar()
plt.show()
imgplot.figure.savefig('%s.pdf'%filename)
imgplot.write_png('%s.png'%filename)
# http://matplotlib.org/users/image_tutorial.html
# https://github.com/assimp/assimp/blob/master/port/PyAssimp/README.md
# http://grass.osgeo.org/grass70/manuals/r.viewshed.html
