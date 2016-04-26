#! /usr/bin/env python
"""
Will process all cloud.%5i.{raw,pcd} in ${ATLAAS_PATH,.}
Then save tiles and region.png
"""
import sys
import time
import atlaas

begin       = 0     if len(sys.argv) < 2 else int(sys.argv[1])
end         = 2**32 if len(sys.argv) < 3 else int(sys.argv[2])
tile_size   = 60.0  if len(sys.argv) < 4 else float(sys.argv[3])
pixel_size  = 0.1   if len(sys.argv) < 5 else float(sys.argv[4])
utm_x       = 0.0   if len(sys.argv) < 6 else float(sys.argv[5])
utm_y       = 0.0   if len(sys.argv) < 7 else float(sys.argv[6])
utm_z       = 0.0   if len(sys.argv) < 8 else float(sys.argv[7])
utm_zone    = 31    if len(sys.argv) < 9 else int(sys.argv[8])
utm_north   = True  if len(sys.argv) < 10 else bool(sys.argv[9])

test = atlaas.Atlaas()
test.init(tile_size, tile_size, pixel_size, utm_x, utm_y, utm_z, utm_zone, utm_north)

start = time.time()
npcds = test.process(begin, end)
total = time.time() - start

start = time.time()
test.region('region.png')
print( "process region    %.3f ms" % (1000. * (time.time() - start)))
print( "process cloud avg %.3f ms" % (1000. * total / npcds) )

# gdal_merge.py -co "COMPRESS=DEFLATE" atlaas.*x*.tif
# gdaldem hillshade -b 4 out.tif -of PNG out.hillshade.png
# xdg-open out.hillshade.png
