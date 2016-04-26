#! /usr/bin/env python
"""
matlaas
=======

*multi-atlaas, a robots communication framework*

usage:   matlaas.py x0 y0 x1 y1
example: matlaas.py 123.4 567.8 -98.7 -65.4

MATLAAS_LIST is an environment variable containing ';' separated list of URL:
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
import math
import socket
import logging
from urllib import urlopen, urlretrieve # urllib.request in Python 3
from lxml import etree
from lxml.html import parse
import gdal
import atlaas
from atlaas.region import merge_or_copy
from atlaas.helpers.bresenham import line
socket.setdefaulttimeout(3)

atlaas_path = os.environ.get('ATLAAS_PATH', '.')
matlaas_list = os.environ.get('MATLAAS_LIST', '')

matlaas_list = matlaas_list.split(';')
while '' in matlaas_list: matlaas_list.remove('')

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

# TODO instead of wating for 404, parse index as:
def index(host='http://127.0.0.1:8000/', xpath='/html/body/ul/li/a'):
    """
    XPath can be:
    * '/html/body/table/tr/td/a' for Apache
    * '/html/body/ul/li/a' for python SimpleHTTPServer
    """
    return [elt.text for elt in parse(host).xpath(xpath)]

def get(url):
    try:
        urlobj = urlopen(url)
    except IOError as err:
        logger.warning("io error [%s] %s"%(url, str(err)))
        return -521, 0
    if urlobj.code != 200:
        logger.warning("http error [%s] %i"%(url, urlobj.code))
        return -urlobj.code, 0
    xml = etree.parse(urlobj)
    elt = xml.xpath('/PAMDataset/Metadata/MDI[@key="COVERAGE"]')
    cov = 0 if not elt else float(elt[0].text)
    elt = xml.xpath('/PAMDataset/Metadata/MDI[@key="AVGALPHA"]')
    avg = 0 if not elt else float(elt[0].text)
    return cov, avg

def process(tiles):
    logger.debug("process %s"%str(tiles))
    if not matlaas_list:
        logger.error("no host list available. export MATLAAS_LIST")
        return False
    data = {}
    for host in matlaas_list:
        for XxY in tiles:
            coverage, avgalpha = get(tile(XxY, host)+".aux.xml")
            if coverage == -404:
                continue # does not exists, try next tile
            elif coverage < 0:
                break # server down, dont try to get other tile from it for now
            logger.debug("tile %s on %s coverage is %f"%(XxY, host, coverage))
            if coverage > 0.01 and (XxY not in data or data[XxY][1] < coverage):
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
                merge_or_copy(filepath, tile(XxY))
                data[XxY][2] = True # mark tile as merged
            else:
                logger.warning("coverage missmatch, discard tile [%s]"%XxY)
            os.remove(filepath)
            os.remove(filemeta)
        except Exception as e:
            logger.error("tile %s error: %s"%(XxY, str(e)))
    return data

graph = None
try:
    from atlaas.helpers.gladys2 import gladys2
    try:
        graph = gladys2("%s/region.png"%atlaas_path,
                        "%s/robot.json"%atlaas_path)
    except RuntimeError as err:
        logger.error(err)
except ImportError:
    logger.error("could not import gladys, install or fix PYTHONPATH")

tile_size = int(os.getenv('ATLAAS_TILE_SIZE', 400)) # tile.width = tile.height
tile_scale = 0.1 # scale_x = scale_y
size = tile_size * tile_scale # tile size in meters
p2t = lambda val: int(math.floor(val/size))

def tiles_for_path(a, b, res=[], cost=0):
    l2i = lambda l: [int(v) for v in l]
    if graph:
        cost, path = graph.path(a, b)
        if path:
            # Bresenham res[-1] -> b
            res += path + list(line(*l2i(path[-1]+b)))
    res += list(line(*l2i(a+b))) # Bresenham a -> b
    logger.debug(res)
    return cost, set(['%ix%i'%(p2t(x), p2t(-y)) for x,y in res])

def main(argv=[]):
    if len(argv) == 2:
        bounds = int(argv[1])
        tilesrange = range(-bounds, bounds)
        tiles = ['%ix%i'%(x,y) for x in tilesrange for y in tilesrange]
    else:
        if len(argv) < 4:
            print(__doc__)
            return 1
        x0, y0, x1, y1 = [float(arg) for arg in argv[1:5]]
        cost, tiles = tiles_for_path((x0,y0), (x1, y1))
    data = process(tiles)
    print(data)
    if data:
        atlaas.merge("region.*x*.png", "region.png")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
