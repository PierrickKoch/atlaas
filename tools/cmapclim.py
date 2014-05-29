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

def convert(fin, fout, cmin, cmax, cmap=cm.spectral):
    geo  = gdal.Open(fin)
    img  = geo.ReadAsArray()
    # filter out NoData ( -10000 ) using np magic
    img[img == -10000] = np.nan
    img = (img - cmin) * (1./(cmax - cmin))
    img[np.isnan(img)] = 0
    # in case of JPEG or WebP, set quality to 90%, else this option is ignored
    Image.fromarray(np.uint8(cmap(img)*255)).save(fout, quality=90)

def main(argv):
    if len(argv) < 3:
        sys.stderr.write(__doc__%argv[0])
        return 1

    cmin = float(argv[2])
    cmax = float(argv[3])
    fout = '%s.jpg'%argv[1]
    convert(argv[1], fout, cmin, cmax)

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
