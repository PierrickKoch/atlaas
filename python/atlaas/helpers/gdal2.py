import numpy
import gdal

class gdal2:
    def __init__(self, filepath):
        self.geodata = gdal.Open( filepath )
        bn = [self.geodata.GetRasterBand(i).GetMetadata().get('NAME', str(i)) \
              for i in range(1, self.geodata.RasterCount+1)]
        self.filepath  = filepath
        self.bands     = self.geodata.ReadAsArray()
        self.names     = {v:k for k,v in enumerate(bn)}
        self.transform = self.geodata.GetGeoTransform()
        self.scale_x   = self.transform[1]
        self.scale_y   = self.transform[5]
        self.utm_x     = self.transform[0]
        self.utm_y     = self.transform[3]
        self.meta      = self.geodata.GetMetadata()
        self.custom_x_origin = float(self.meta.get('CUSTOM_X_ORIGIN', 0))
        self.custom_y_origin = float(self.meta.get('CUSTOM_Y_ORIGIN', 0))
    def u2p(self, x, y):
        return ((x - self.utm_x) / self.scale_x,
                (y - self.utm_y) / self.scale_y)
    def p2u(self, x, y):
        return (x * self.scale_x + self.utm_x,
                y * self.scale_y + self.utm_y)
    def c2p(self, x, y):
        return self.u2p(*self.c2u(x, y))
    def p2c(self, x, y):
        return self.u2c(*self.p2u(x, y))
    def c2u(self, x, y):
        return (x + self.custom_x_origin,
                y + self.custom_y_origin)
    def u2c(self, x, y):
        return (x - self.custom_x_origin,
                y - self.custom_y_origin)
    def save(self, filepath=None):
        if filepath is None:
            filepath = self.filepath
        driver = self.geodata.GetDriver()
        out = driver.CreateCopy(filepath, self.geodata)
        if len(self.bands.shape) == 2:
            band = out.GetRasterBand(1)
            band.WriteArray(self.bands)
        elif len(self.bands.shape) == 3:
            for i_band in range(self.bands.shape[0]):
                band = out.GetRasterBand(i_band+1)
                band.WriteArray(self.bands[i_band])
        else:
            raise TypeError('bands.shape: %s'%str(self.bands.shape))
        out.FlushCache()
    def dtm(self, variance_threshold=0.01):
        bn = self.bands[self.names['N_POINTS']]
        bv = self.bands[self.names['VARIANCE']]
        be = self.bands[self.names['Z_MEAN']]
        ba = self.bands[self.names['Z_MAX']]
        bd = numpy.where(bv > variance_threshold, ba, be)
        bd[bn < 1] = numpy.nan
        return bd
