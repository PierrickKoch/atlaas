import gladys

class gladys2:
    def __init__(self, region, robot):
        self.ctmap = gladys.costmap(region, robot)
        self.graph = gladys.nav_graph(self.ctmap)
        self.gdmap = self.ctmap.get_map()
    def u2p(self, x, y):
        return gladys.point_utm2pix(self.gdmap, x, y)
    def p2u(self, x, y):
        return gladys.point_pix2utm(self.gdmap, x, y)
    def c2p(self, x, y):
        return gladys.point_custom2pix(self.gdmap, x, y)
    def p2c(self, x, y):
        return gladys.point_pix2custom(self.gdmap, x, y)
    def c2u(self, x, y):
        return gladys.point_custom2utm(self.gdmap, x, y)
    def u2c(self, x, y):
        return gladys.point_utm2custom(self.gdmap, x, y)
    def path(self, a, b):
        # A* a, b
        res_utm, cost = self.graph.search_with_cost(self.c2u(*a), self.c2u(*b))
        res = [self.u2c(x, y) for x, y in res_utm]
        return cost, res
    def map(self):
        # arr = np.array(data).reshape(height, width)
        return {
            'data': self.gdmap.get_band('WEIGHT'),
            'width': self.gdmap.get_width(),
            'height': self.gdmap.get_height(),
            'scale_x': self.gdmap.get_scale_x(),
            'scale_y': self.gdmap.get_scale_y(),
            'utm_x': self.gdmap.get_utm_pose_x(),
            'utm_y': self.gdmap.get_utm_pose_y(),
            'custom_x_origin': self.gdmap.get_custom_x_origin(),
            'custom_y_origin': self.gdmap.get_custom_y_origin(),
        }
    def map_numpy(self):
        import numpy
        return numpy.array(self.gdmap.get_band('WEIGHT'), 'float32').reshape(
            self.gdmap.get_height(), self.gdmap.get_width())
    def map_gdal(self, filename):
        ''' Same as:
        m = self.map()
        import gdal, osr, numpy
        driver = gdal.GetDriverByName('GTiff')
        dst_ds = driver.Create( filename, m['width'], m['height'], 1,
            gdal.GDT_Float32 )
        dst_ds.SetGeoTransform( [ m['utm_x'], m['scale_x'], 0,
            m['utm_y'], 0, m['scale_y'] ] )
        dst_ds.SetMetadataItem('CUSTOM_X_ORIGIN', m['custom_x_origin'])
        dst_ds.SetMetadataItem('CUSTOM_Y_ORIGIN', m['custom_y_origin'])
        srs = osr.SpatialReference()
        srs.SetUTM( 31, 1 ) # FIXME
        srs.SetWellKnownGeogCS( 'WGS84' )
        dst_ds.SetProjection( srs.ExportToWkt() )
        arr = numpy.array(m['data'], 'float32').reshape(m['height'], m['width'])
        dst_ds.GetRasterBand(1).WriteArray( arr )
        dst_ds.FlushCache()
        return dst_ds
        '''
        import gdal
        self.gdmap.save(filename)
        return gdal.Open(filename)
