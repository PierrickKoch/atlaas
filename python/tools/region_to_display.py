#! /usr/bin/env python
"""
tile_to_region
==============

*transform dtm tiles to region-map tiles*

usage: python tile_to_region.py
"""
import os
import sys
import time
import glob
from atlaas.helpers.image import save, load
# https://raw.githubusercontent.com/BIDS/colormap/master/colormaps.py
from colormaps import viridis
# initialize the logger
import logging
logger = logging.getLogger(__name__) # or __file__
handler = logging.StreamHandler()
handler.setFormatter( logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s') )
logger.addHandler( handler )
logger.setLevel(logging.DEBUG)

def run():
    mtime = {}
    needup = []
    region_tiles = glob.glob("region.*x*.png")
    # first pass get all tile st_mtime
    for fregion in region_tiles:
        XxY = fregion.split('.')[1]
        sregion = os.stat(fregion)
        mtime[XxY] = sregion.st_mtime
        fprecis = "display.%s.precis.png"%XxY
        if os.path.isfile(fprecis):
            sprecis = os.stat(fprecis)
            if sregion.st_mtime <= sprecis.st_mtime:
                continue
        needup.append(XxY)
    mtime_min = min(mtime.values())
    mtime_max = max(mtime.values())
    mtime_fac = 1./(mtime_max - mtime_min)
    mtime_col = lambda mt: (mt - mtime_min) * mtime_fac
    for XxY in needup:
        img = load("region.%s.png"%XxY)
        alpha = img[:,:,1]
        save("display.%s.precis.png"%XxY, (viridis(alpha)*255).astype('uint8'))
        alpha.fill(mtime_col(mtime[XxY])*255)
        save("display.%s.mtime.png"%XxY, (viridis(alpha)*255).astype('uint8'))

if __name__ == '__main__':
    tick = 1 if len(sys.argv) < 2 else float(sys.argv[1])
    while 1:
        run()
        time.sleep(tick)
