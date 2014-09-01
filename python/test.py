import atlaas
import numpy as np

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

transformation = np.identity(4, dtype=np.double)

def merge_test():
    """
    >>> cloud = np.array([
    ...     [ 1, 2, 3, 0],
    ...     [-1,-2,-3, 0],
    ... ], dtype=np.float32)
    >>> 
    >>> test.merge(cloud, transformation)
    >>> 
    >>> cloud = np.array([
    ...     [ 1, 2, 3],
    ...     [-1,-2,-3],
    ... ], dtype=np.float32)
    >>> 
    >>> test.merge(cloud, transformation)
    >>> 
    >>> cloud = np.array([
    ...     [ 1, 2, 3, 0, 5],
    ...     [-1,-2,-3, 0, 5],
    ... ], dtype=np.float32)
    >>> 
    >>> test.merge(cloud, transformation)
    Traceback (most recent call last):
        ...
    TypeError: array shape[1] must be 3 or 4, cloud: XYZ[I]
    >>> 
    >>> cloud = np.array([
    ...     [ 1, 2],
    ...     [-1,-2],
    ... ], dtype=np.float32)
    >>> 
    >>> test.merge(cloud, transformation)
    Traceback (most recent call last):
        ...
    TypeError: array shape[1] must be 3 or 4, cloud: XYZ[I]
    """
    return

if __name__ == "__main__":
    import doctest
    doctest.testmod()
