import atlaas
import numpy as np

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

transformation = np.array([
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1,
], dtype=np.double)

cloud = np.array([
    [ 1, 2, 3, 0],
    [-1,-2,-3, 0],
], dtype=np.float32)

test.merge(cloud, transformation)
