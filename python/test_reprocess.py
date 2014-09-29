import time
import atlaas
import numpy as np
test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
transformation = np.identity(4, dtype=np.double)
cloud = np.random.rand(2000, 3).astype('float32')
time_ms = []
for i in range(100):
  time_ms.append( int(time.time()*1000) )
  transformation[0][3]=i
  transformation[1][3]=i
  test.merge(cloud, transformation)

test.save_currents()

npcds = test.reprocess(time_ms[10], time_ms[30], 20, 80)

print(time_ms)
print(npcds)

#c = [t for t,xy in b]
#[ea-ec for ea,ec in zip(a,c)]
