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
import gdal
import numpy
import Image # aka PIL, because gdal PNG driver does not support WriteBlock

# initialize the logger
import logging
logger = logging.getLogger(__name__) # or __file__
handler = logging.StreamHandler()
handler.setFormatter( logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s') )
logger.addHandler( handler )
logger.setLevel(logging.DEBUG)

devnull = open(os.devnull, 'w')
stdout = sys.stdout

def merge(filetmp, filedst):
    gtmp = gdal.Open(filetmp)
    gdst = gdal.Open(filedst)
    r1 = gtmp.ReadAsArray()
    r2 = gdst.ReadAsArray()
    # compute cost from r1 and r2 ([0]: cost, [1]: precision)
    # cost is cost_r1 where precision_r1 > precision_r2 else cost_r2
    cost = numpy.where(r1[1] > r2[1], r1[0], r2[0])
    alph = numpy.where(r1[1] > r2[1], r1[1], r2[1])
    image = Image.fromarray(cost)
    ialph = Image.fromarray(alph)
    image.putalpha(ialph)
    image.save(filedst)
    # update COVERAGE metadata
    coverage = alph[alph > 0].size / float(alph.size)
    logger.debug("merge coverage gain: %.3f" %
        (coverage - float(gdst.GetMetadata().get('COVERAGE', '0'))))
    gdst.SetMetadataItem('COVERAGE', str(coverage))

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
    tick = 1 if len(sys.argv) < 2 else float(sys.argv[1])
    while 1:
        run()
        time.sleep(tick)
