# -*- coding: utf-8 -*-
# <nbformat>2</nbformat>

# <codecell>


filename = 'out.tif'

import gdal
# run `ipython notebook --pylab inline`
import matplotlib.pyplot as plt
names = {v:k for k,v in enumerate(["N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "VARIANCE", "TIME"])}
geotiff = gdal.Open( filename )
bands   = geotiff.ReadAsArray()

# <codecell>

band = bands[names['N_POINTS']]
plt.subplots(figsize=(10, 10))
plt.semilogy()
plt.hist( band.flatten(), 256 )
plt.show()

# <codecell>

band = bands[names['N_POINTS']]
plt.subplots(figsize=(14, 14))
imgplot = plt.imshow( band )
imgplot.set_clim(0, 500)
plt.colorbar()
plt.show()

# <codecell>

band = bands[names['VARIANCE']]
plt.subplots(figsize=(10, 10))
plt.semilogy()
plt.hist( band.flatten(), 256 )
plt.show()

# <codecell>

band = bands[names['VARIANCE']]
plt.subplots(figsize=(14, 14))
imgplot = plt.imshow( band )
imgplot.set_clim(0, 0.2)
imgplot.set_cmap('spectral')
plt.colorbar()
plt.show()

# <codecell>

band = bands[names['Z_MEAN']]
plt.subplots(figsize=(10, 10))
plt.semilogy()
plt.hist( band.flatten(), 256 )
plt.show()

# <codecell>

band = bands[names['Z_MEAN']]
plt.subplots(figsize=(14, 14))
imgplot = plt.imshow( band )
imgplot.set_clim(-0.5, 4.0)
plt.colorbar()
plt.show()

# <markdowncell>

# - http://nbviewer.ipython.org/9596093
# - http://matplotlib.org/users/image_tutorial.html
# - https://github.com/assimp/assimp/blob/master/port/PyAssimp/README.md
# - http://grass.osgeo.org/grass70/manuals/r.viewshed.html
