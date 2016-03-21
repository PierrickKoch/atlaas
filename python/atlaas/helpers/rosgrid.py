import numpy
from atlaas.helpers.gdal2 import gdal2
from nav_msgs.msg import OccupancyGrid

def atlaas8u_grid(filepath, stamp=None, frame_id='/map'):
    g = gdal2(filepath)
    b = g.bands[0] if len(g.bands.shape) > 2 else g.bands
    og = OccupancyGrid()
    og.data = (b.astype('float')/2.55).astype('uint8').flatten()
    if stamp: og.header.stamp = stamp
    og.header.frame_id = frame_id
    og.info.resolution = g.scale_x
    og.info.height, og.info.width = b.shape
    og.info.origin.position.x, og.info.origin.position.y = g.u2c(g.utm_x, g.utm_y)
    og.info.origin.orientation.x = 1 # flip to transform UTM-ROS (scale_y < 0)
    return og

def atlaas_grid(filepath, var_threshold=0.1, stamp=None, frame_id='/map'):
    """ Usage:
    atlaas.merge("atlaas.*x*.tif", "out.tif")
    if os.path.isfile("out.tif"):
        atlaas_grid("out.tif")
    """
    g = gdal2(filepath)
    bnp = g.bands[g.names['N_POINTS']]
    bma = g.bands[g.names['Z_MAX']]
    bme = g.bands[g.names['Z_MEAN']]
    bva = g.bands[g.names['VARIANCE']]
    data = numpy.where(bva > var_threshold, bma, bme)
    delta = data.max() - data.min()
    ddis = (100.0*(data-data.min())/delta).astype('uint8')
    ddis[bnp < 1] = 0 # no points, unknown
    og = OccupancyGrid()
    og.data = ddis.flatten()
    if stamp: og.header.stamp = stamp
    og.header.frame_id = frame_id
    og.info.resolution = g.scale_x
    og.info.height, og.info.width = data.shape
    og.info.origin.position.x, og.info.origin.position.y = g.u2c(g.utm_x, g.utm_y)
    og.info.origin.orientation.x = 1 # flip to transform UTM-ROS (scale_y < 0)
    return og
