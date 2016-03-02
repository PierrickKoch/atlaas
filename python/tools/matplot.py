# coding: utf-8

get_ipython().magic(u'matplotlib inline')
filename = 'out.tif'

from atlaas.helpers.gdal2 import gdal2
from atlaas.helpers.matplot import show, hist

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
