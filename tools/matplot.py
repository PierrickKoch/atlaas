# coding: utf-8

get_ipython().magic(u'matplotlib inline')
filename = 'out.tif'

import gdal
import numpy as np
import matplotlib.pyplot as plt

class gdal2:
    def __init__(self, filepath):
        self.geodata = gdal.Open( filepath )
        bn = [self.geodata.GetRasterBand(i).GetMetadata().get('NAME', str(i)) \
              for i in range(1, self.geodata.RasterCount+1)]
        self.filepath  = filepath
        self.bands     = self.geodata.ReadAsArray()
        self.names     = {v:k for k,v in enumerate(bn)}
        self.transform = self.geodata.GetGeoTransform()
        self.scale_x   = self.transform[1]
        self.scale_y   = self.transform[5]
        self.utm_x     = self.transform[0]
        self.utm_y     = self.transform[3]
        self.meta      = self.geodata.GetMetadata()
        self.custom_x_origin = float(self.meta.get('CUSTOM_X_ORIGIN', 0))
        self.custom_y_origin = float(self.meta.get('CUSTOM_Y_ORIGIN', 0))
    def u2p(self, x, y):
        return ((x - self.utm_x) / self.scale_x,
                (y - self.utm_y) / self.scale_y)
    def p2u(self, x, y):
        return (x * self.scale_x + self.utm_x,
                y * self.scale_y + self.utm_y)
    def c2p(self, x, y):
        return self.u2p(*self.c2u(x, y))
    def p2c(gdal, x, y):
        return self.u2c(*self.p2u(x, y))
    def c2u(self, x, y):
        return (x + self.custom_x_origin,
                y + self.custom_y_origin)
    def u2c(self, x, y):
        return (x - self.custom_x_origin,
                y - self.custom_y_origin)

def hist(data, fsize=(12, 5)):
    plt.figure(figsize = fsize)
    plt.hist( data.flatten(), 100, log=1 )

def show(data, cmin=0, cmax=1, cmap='viridis', fsize=(14, 12), x=[], y=[]):
    plt.figure(figsize = fsize)
    imgplot = plt.imshow( data, interpolation='none' )
    imgplot.set_clim(cmin, cmax)
    imgplot.set_cmap(cmap)
    plt.colorbar()
    if len(x) and len(y):
        plt.plot(x, y, 'r', scalex=False, scaley=False)
    plt.tight_layout(pad=0)

g = gdal2(filename)

# <codecell>

band = g.bands[g.names['N_POINTS']]
hist(band)

# <codecell>

show(band, 0, 100)

# <codecell>

band = g.bands[g.names['VARIANCE']]
hist(band)

# <codecell>

show(band, 0, 0.2)

# <codecell>

band = g.bands[g.names['Z_MEAN']]
hist(band)

# <codecell>

show(band, 0, 10.0)

# <codecell>

band = g.bands[g.names['TIME']]
hist(band)

# <codecell>

show(band, 0, 200)

# <codecell>

band = g.bands[g.names['DIST_SQ']]
hist(band)

# <codecell>

show(band, 0, 550)

# <markdowncell>

# - http://nbviewer.ipython.org/9596093
# - http://matplotlib.org/users/image_tutorial.html
# - https://github.com/assimp/assimp/blob/master/port/PyAssimp/README.md
# - http://grass.osgeo.org/grass70/manuals/r.viewshed.html
