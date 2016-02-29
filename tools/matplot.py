# coding: utf-8

get_ipython().magic(u'matplotlib inline')
filename = 'out.tif'

import gdal
import numpy as np
import matplotlib.pyplot as plt

class gdal2:
    def __init__(self, filepath):
        self.filepath = filepath
        self.geodata = gdal.Open( filepath )
        self.bands = self.geodata.ReadAsArray()
        bn = [self.geodata.GetRasterBand(i).GetMetadata().get('NAME', str(i)) \
              for i in range(1, self.geodata.RasterCount+1)]
        self.names = {v:k for k,v in enumerate(bn)}
        self.transform = geodata.GetGeoTransform()
        self.meta      = geodata.GetMetadata()
        self.scale_x = self.transform[1]
        self.scale_y = self.transform[5]
        self.utm_x   = self.transform[0]
        self.utm_y   = self.transform[3]
        self.custom_x_origin = float(self.meta['CUSTOM_X_ORIGIN'])
        self.custom_y_origin = float(self.meta['CUSTOM_Y_ORIGIN'])

    def u2p(self, x, y):
        return ((x - self.utm_x) / self.scale_x,
                (y - self.utm_y) / self.scale_y)
    def p2u(self, x, y):
        return (x * self.scale_x + self.utm_x,
                y * self.scale_y + self.utm_y)
    def c2p(self, x, y):
        return u2p(*self.c2u(x, y))
    def p2c(gdal, x, y):
        return u2c(*self.p2u(x, y))
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
