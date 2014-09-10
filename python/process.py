#! /usr/bin/env python
import time
import atlaas

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

start = time.time()
npcds = test.process()
total = time.time() - start

test.save_currents()

print( "process avg %.3f ms" % (1000. * total / npcds) )

# gdal_merge.py -co "COMPRESS=DEFLATE" atlaas.*x*.tif
# gdaldem hillshade -b 4 out.tif -of PNG out.hillshade.png
# xdg-open out.hillshade.png
