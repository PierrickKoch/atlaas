#! /usr/bin/env python
"""
tile_to_region
==============

*transform dtm tiles to region-map tiles*

usage: while true; do python tile_to_region.py; sleep 1; done
"""
import os
import glob
import atlaas

need_merge = False
atlaas_tiles = glob.glob("atlaas.*x*.tif")
for fatlaas in atlaas_tiles:
    fregion = "region.%s.png"%fatlaas.split('.')[1]
    need_convert = True
    if os.path.isfile(fregion):
        satlaas = os.stat(fatlaas)
        sregion = os.stat(fregion)
        if satlaas.st_mtime <= sregion.st_mtime:
            need_convert = False
    if need_convert:
        print(fregion)
        atlaas.tile_to_region(fatlaas, fregion)
        need_merge = True

if need_merge:
    atlaas.merge("region.*x*.png", "region.png")
