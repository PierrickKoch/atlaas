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
import gdal
import Image
import numpy as np
from matplotlib import cm

names = {v:k for k,v in enumerate(["N_POINTS", "Z_MIN", "Z_MAX", "Z_MEAN", "VARIANCE", "TIME", "DIST_SQ"])}

def convert(fin, fout, cmin, cmax, cmap=cm.spectral, npt=None):
    geo = gdal.Open(fin)
    img = geo.ReadAsArray() # get band as a numpy.array
    if len(img.shape) > 2: # multi-layer
        npt = img[names["N_POINTS"]]
        img = img[names["Z_MEAN"]]
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    img[npt < 1] = 0
    # in case of JPEG or WebP, set quality to 90%, else this option is ignored
    Image.fromarray(np.uint8(cmap(img)*255)).save(fout, quality=90)

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
