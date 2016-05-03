#! /usr/bin/env python
import os
import glob
import atlaas
from atlaas.helpers.gdal2 import gdal2
from atlaas.helpers.image import zero

if not glob.glob("atlaas.*x*.tif"):
    test = atlaas.Atlaas()
    test.init(120.0, 120.0, 0.1, 377016.5, 4824342.9, 141.0, 31, True)
    test.process()
    test.save_currents()
    del test

# 1st erase data in tiles so as to have same size in merge() without noise
for fatlaas in glob.glob("atlaas.*x*.tif"):
    g = gdal2(fatlaas)
    g.bands *= 0
    g.save()
    fregion = "region.%s.png"%fatlaas.split('.')[1]
    atlaas.tile_to_region(fatlaas, fregion)

for fregion in glob.glob("region.*x*.png"):
    zero(fregion) # empty data

pcd_id = 0
patern = 'cloud.%05i.pcd'

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 377016.5, 4824342.9, 141.0, 31, True)

while os.path.isfile(patern%pcd_id):
    test.merge_file(patern%pcd_id)
    test.save_currents()
    atlaas.merge('atlaas.*x*.tif', 'merged.atlaas.%05i.tif'%pcd_id, compress=True)
    for fatlaas in glob.glob("atlaas.*x*.tif"):
        fregion = "region.%s.png"%fatlaas.split('.')[1]
        if os.stat(fatlaas).st_mtime > os.stat(fregion).st_mtime:
            atlaas.tile_to_region(fatlaas, fregion)
    atlaas.merge('region.*x*.png', 'merged.region.%05i.png'%pcd_id)
    pcd_id += 1
