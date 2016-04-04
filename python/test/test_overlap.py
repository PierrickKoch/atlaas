import atlaas
import numpy as np

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
transformation = np.identity(4, dtype=np.double)
cloud = np.random.rand(2000, 3).astype('float32') * 100

for i in range(100):
    transformation[0:2,3] = i
    test.merge(cloud, transformation)

res = test.pcd_overlap(99)
print(res)
overlap = {int(k):float(v) for k,v in [it.split(':') for it in res[1:-1].split(',')]}
print(overlap)
