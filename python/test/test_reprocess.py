import time
import atlaas
import numpy as np

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
transformation = np.identity(4, dtype=np.double)
cloud = np.random.rand(2000, 3).astype('float32')
covariance = np.array([0]*36, dtype=np.float64).reshape(6, 6)

time_ms = []
for i in range(100):
    time_ms.append( int(time.time()*1000) )
    transformation[0:2,3] = i
    test.merge(cloud, transformation, covariance)

start = time.time()
npcds = test.reprocess(time_ms[10], time_ms[30], 20, 80)
total = time.time() - start

test.save_currents()

print( "process avg %.3f ms" % (1000. * total / npcds) )
