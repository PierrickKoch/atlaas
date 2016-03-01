#! /usr/bin/env python
"""
Do something like matplotlib's:
geo  = gdal.Open( filename )
band = geotiff.ReadAsArray()
imgplot = plt.imshow( band )
imgplot.set_clim(cmin, cmax)
imgplot.set_cmap(cmap)

in place, keeping image's resolution.

usage: %s input.tif cmin cmax
"""
import sys
from atlaas.helpers.gdal2 import gdal2
from atlaas.helpers.image import save
from matplotlib import cm
if 'viridis' in dir(cm):
    colormap = cm.viridis
else:
    colormap = cm.spectral

def convert(fin, fout, cmin, cmax, cmap=colormap, npt=None):
    geo = gdal2(fin)
    img = geo.bands
    if len(img.shape) > 2: # multi-layer
        npt = geo.bands[geo.names["N_POINTS"]]
        img = geo.bands[geo.names["Z_MEAN"]]
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    img[npt < 1] = 0
    # in case of JPEG or WebP, set quality to 90%, else this option is ignored
    save(fout, (cmap(img)*255).astype('uint8'))

def main(argv):
    if len(argv) < 4:
        sys.stderr.write(__doc__%argv[0])
        return 1

    cmin = float(argv[2])
    cmax = float(argv[3])
    fout = '%s.jpg'%argv[1]
    convert(argv[1], fout, cmin, cmax)

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
