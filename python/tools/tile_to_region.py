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
import atlaas
from atlaas.region import merge

# initialize the logger
import logging
logger = logging.getLogger(__name__) # or __file__
handler = logging.StreamHandler()
handler.setFormatter( logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s') )
logger.addHandler( handler )
logger.setLevel(logging.DEBUG)

devnull = open(os.devnull, 'w')
stdout = sys.stdout

def run():
    need_merge = False
    atlaas_tiles = glob.glob("atlaas.*x*.tif")
    for fatlaas in atlaas_tiles:
        fregion = "region.%s.png"%fatlaas.split('.')[1]
        need_convert = True
        file_exists = False
        if os.path.isfile(fregion):
            file_exists = True
            satlaas = os.stat(fatlaas)
            sregion = os.stat(fregion)
            if satlaas.st_mtime <= sregion.st_mtime:
                need_convert = False
        if need_convert:
            logger.debug(fregion)
            if file_exists:
                atlaas.tile_to_region(fatlaas, fregion+".tmp")
                merge(fregion+".tmp", fregion)
                os.unlink(fregion+".tmp")
            else:
                atlaas.tile_to_region(fatlaas, fregion)
            need_merge = True
    if need_merge:
        sys.stdout = devnull
        atlaas.merge("region.*x*.png", "region.png")
        sys.stdout = stdout

if __name__ == '__main__':
    if 'once' in sys.argv:
        run()
        sys.exit(0)
    tick = 1 if len(sys.argv) < 2 else float(sys.argv[1])
    while 1:
        run()
        time.sleep(tick)
