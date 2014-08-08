import atlaas

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

transformation = [
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1,
]
cloud = [
    [ 1, 2, 3, 0],
    [-1,-2,-3, 0],
]

test.merge(cloud, transformation)
