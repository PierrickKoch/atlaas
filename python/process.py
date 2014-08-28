#! /usr/bin/env python
import time
import atlaas

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

start = time.time()
npcds = test.process_pcd()
total = time.time() - start

test.save_currents()

print( "process avg %.3f ms" % (1000. * total / npcds) )
