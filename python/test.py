import atlaas
import numpy as np

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

transformation = np.identity(4, dtype=np.double)

def merge_test():
    """
    >>> cloud = np.random.rand(200000, 4).astype('float32')
    >>> test.merge(cloud, transformation)

    >>> cloud = np.random.rand(200000, 5).astype('float32')
    >>> test.merge(cloud, transformation)
    Traceback (most recent call last):
        ...
    TypeError: array shape[1] must be 3 or 4, cloud: XYZ[I]

    >>> cloud = np.random.rand(200000, 2).astype('float32')
    >>> test.merge(cloud, transformation)
    Traceback (most recent call last):
        ...
    TypeError: array shape[1] must be 3 or 4, cloud: XYZ[I]

    >>> cloud = np.random.rand(200000, 3)
    >>> test.merge(cloud, transformation)
    Traceback (most recent call last):
        ...
    ValueError: Buffer dtype mismatch, expected 'float32_t' but got 'double'

    >>> cloud = np.random.rand(200000, 3).astype('float32')
    >>> test.merge(cloud, transformation)

    """
    return

def save_test():
    """
    >>> filepath = 'cloud.pcd'
    >>> cloud = np.random.rand(200000, 4).astype('float32')
    >>> atlaas.save(filepath, cloud, transformation)
    >>> tr, cd = atlaas.load(filepath)
    >>> np.allclose(cd, cloud)
    True

    """
    return

if __name__ == "__main__":
    import doctest
    doctest.testmod()
