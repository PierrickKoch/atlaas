#! /usr/bin/env python
"""
pypero
======

*a robot communication framework*

usage:   pypero.py x0 y0 x1 y1
example: pypero.py 123.4 567.8 -98.7 -65.4

PYPERO_LIST is an environment variable containing ';' separated list of URL:
protocol://[user[:pass]@]host[:port][/folder]

of an HTTP or FTP server runnning in ATLAAS_PATH.

Assuming all robots (servers) have the same "CUSTOM_ORIGINE",
ie. the same tiles space frame.

At the moment we also assume that tile size and resolution are the same for all
robots.

ATLAAS_PATH is the local absolute path to atlaas tiles.

A basic webserver is provided by the command: python -m SimpleHTTPServer
"""
import os
import sys
import json
import shutil
import socket
import logging
from urllib import urlopen, urlretrieve # urllib.request in Python 3
from lxml import etree
import gdal
import numpy
import Image # aka PIL, because gdal PNG driver does not support WriteBlock
from math import floor

atlaas_path = os.environ.get('ATLAAS_PATH', '.')
pypero_list = os.environ.get('PYPERO_LIST', '')

pypero_list = pypero_list.split(';')
while '' in pypero_list: pypero_list.remove('')

# initialize the logger
logger = logging.getLogger(__name__) # or __file__
handler = logging.StreamHandler()
handler.setFormatter( logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s') )
logger.addHandler( handler )
logger.setLevel(logging.DEBUG)

def tile(XxY, base=atlaas_path):
    return "%s/region.%s.png" % (base, XxY)

def check(filepath):
    geotiff = gdal.Open(filepath)
    npoints = geotiff.GetRasterBand(2).ReadAsArray()
    return npoints[npoints>0].size / float(npoints.size)

def get(url):
    xml = etree.parse(urlopen(url)) # urlopen can deal with SSL
    elt = xml.xpath('/PAMDataset/Metadata/MDI[@key="COVERAGE"]')
    return -1 if not elt else float(elt[0].text)

def merge(filetmp, filedst):
    if os.path.isfile(filedst):
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
    else: # copy temp to dest
        # os.rename dont work across filesystem
        #   [Errno 18] Invalid cross-device link
        shutil.copy(filetmp, filedst)
        shutil.copy(filetmp+".aux.xml", filedst+".aux.xml")

def process(tiles):
    logger.debug("process %s"%str(tiles))
    if not pypero_list:
        logger.error("no host list available. export PYPERO_LIST")
        return False
    data = {}
    for host in pypero_list:
        for XxY in tiles:
            try:
                coverage = get(tile(XxY, host)+".aux.xml")
            except etree.XMLSyntaxError: # probably a 404 html
                logger.info("file doesnt exist on host [%s,%s]"%(XxY, host))
                continue # try next tile
            except IOError as e:
                if 'failed to load HTTP resource' in e.message or \
                   'No such file or directory' in e.message:
                    logger.info("file doesnt exist on host [%s,%s]"%(XxY, host))
                    continue # try next tile
                elif 'failed to load external entity' in e.message or \
                     'Connection refused' in str(e):
                    logger.warning("host down [%s]"%host)
                else:
                    logger.warning("%s : %s"%(str(e), host)) # TODO handle this
                    logger.debug({name: getattr(e, name) for name in dir(e)}) # XXX TMP
                break # dont try to get other tile from it for now
            logger.debug("tile %s on %s coverage is %f"%(XxY, host, coverage))
            if coverage > 0.01 and (XxY not in data or data[XxY][1] > coverage):
                data[XxY] = [host, coverage, False]

    for XxY, value in data.items():
        try:
            filepath, _ = urlretrieve(tile(XxY, value[0]))
            filemeta, _ = urlretrieve(tile(XxY, value[0])+".aux.xml",
                                      filepath+".aux.xml")
            logger.info("got %s [%s] from %s"%(XxY, filepath, value[0]))
            # make sure the file is OK (coverage metadata are equal)
            if abs(check(filepath) - value[1]) < 1e-6:
                # mv filepath -> tile(XxY)
                logger.info("tile %s success moving it [%s]"%(filepath, XxY))
                merge(filepath, tile(XxY))
                data[XxY][2] = True # mark tile as good
            else:
                logger.warning("coverage missmatch, discard tile [%s]"%XxY)
            os.remove(filepath)
            os.remove(filemeta)
        except Exception as e:
            logger.error("tile %s error: %s"%(XxY, str(e)))
    return data

graph = None
try:
    import gladys
    try:
        ctmap = gladys.costmap("%s/region.png"%atlaas_path,
                               "%s/robot.json"%atlaas_path)
        graph = gladys.nav_graph(ctmap)
    except RuntimeError as err:
        logger.error(err)
except ImportError:
    logger.error("could not import gladys, install or fix PYTHONPATH")

tile_size = 400 # tile.width = tile.height
tile_scale = 0.1 # scale_x = scale_y
size = tile_size * tile_scale # tile size in meters
p2t = lambda val: int(floor(val/size))

# need to get tile on straight line as well (Bresenham)
# since gladys does not give points outside region
def line(x0, y0, x1, y1):
    """ Bresenham line from (x0,y0) -> (x1,y1)

    list(line(1,2,3,4))
    >>> [(1, 2), (2, 3), (3, 4)]
    list(line(-1,-2,-3,-4))
    >>> [(-1, -2), (-2, -3), (-3, -4)]
    """
    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep: # swap
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    deltax = abs(x1 - x0)
    deltay = abs(y1 - y0)
    error = deltax / 2
    y = y0
    xstep = 1 if x0 < x1 else -1
    ystep = 1 if y0 < y1 else -1
    for x in range(x0, x1+xstep, xstep):
        yield (y,x) if steep else (x,y)
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax

def tiles_for_path(a, b, graph=None, tra=[], cost=0):
    ta = [p2t(v) for v in a]
    tb = [p2t(v) for v in b]
    if graph: # A* a,b
        res, cost = graph.search_with_cost(a, b)
        tra = ['%ix%i'%(p2t(x), p2t(y)) for x,y in res]
        # Bresenham res[-1] -> b
        lin = list(line(p2t(res[-1][0]), p2t(res[-1][1]), *tb))
        tra += ['%ix%i'%(x, y) for x,y in lin]
    lin = list(line(*ta+tb)) # Bresenham a -> b
    bre = ['%ix%i'%(x, y) for x,y in lin]
    return cost, set(tra+bre)

def main(argv=[]):
    if len(argv) < 4:
        print(__doc__)
        return 1
    x0 = float(argv[1])
    y0 = float(argv[2])
    x1 = float(argv[3])
    y1 = float(argv[4])
    cost, tiles = tiles_for_path((x0,y0), (x1, y1), graph)
    data = process(tiles)
    print(data)
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
