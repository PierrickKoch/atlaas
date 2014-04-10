# -*- coding: utf-8 -*-
# <nbformat>2</nbformat>

# <codecell>

filename = 'out.tif'

import gdal
# run `ipython notebook --pylab inline`
# ipython notebook --pylab inline
import matplotlib.pyplot as plt
names = {v:k for k,v in enumerate(["N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "VARIANCE", "TIME", "DIST_SQ"])}
geotiff = gdal.Open( filename )
bands   = geotiff.ReadAsArray()

def show_hist(name, fsize=(10, 10)):
    band = bands[names[name]]
    plt.subplots(figsize = fsize)
    plt.semilogy()
    plt.hist( band.flatten(), 256 )
    plt.show()

def show_band(name, cmin=0, cmax=1, cmap='spectral', fsize=(14, 14)):
    band = bands[names[name]]
    plt.subplots(figsize = fsize)
    imgplot = plt.imshow( band )
    imgplot.set_clim(cmin, cmax)
    imgplot.set_cmap(cmap)
    plt.colorbar()
    plt.show()

# <codecell>

show_hist('N_POINTS')

# <codecell>

show_band('N_POINTS', 0, 100)

# <codecell>

show_hist('VARIANCE')

# <codecell>

show_band('VARIANCE', 0, 0.2)

# <codecell>

show_hist('Z_MEAN')

# <codecell>

show_band('Z_MEAN', 0, 10.0)

# <codecell>

show_hist('TIME')

# <codecell>

show_band('TIME', 0, 200)

# <codecell>

show_hist('DIST_SQ')

# <codecell>

show_band('DIST_SQ', 0, 550)

# <markdowncell>

# - http://nbviewer.ipython.org/9596093
# - http://matplotlib.org/users/image_tutorial.html
# - https://github.com/assimp/assimp/blob/master/port/PyAssimp/README.md
# - http://grass.osgeo.org/grass70/manuals/r.viewshed.html
