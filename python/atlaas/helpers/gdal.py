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
    def p2c(gdal, x, y):
        return self.u2c(*self.p2u(x, y))
    def c2u(self, x, y):
        return (x + self.custom_x_origin,
                y + self.custom_y_origin)
    def u2c(self, x, y):
        return (x - self.custom_x_origin,
                y - self.custom_y_origin)
