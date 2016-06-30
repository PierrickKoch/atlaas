def line(x0, y0, x1, y1):
    """ Bresenham line from (x0,y0) -> (x1,y1)

    list(line(1,2,3,4))
    >>> [(1, 2), (2, 3), (3, 4)]
    list(line(-1,-2,-3,-4))
    >>> [(-1, -2), (-2, -3), (-3, -4)]
    """
    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep: # swap
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    deltax = abs(x1 - x0)
    deltay = abs(y1 - y0)
    error = deltax / 2
    y = y0
    xstep = 1 if x0 < x1 else -1
    ystep = 1 if y0 < y1 else -1
    for x in range(x0, x1+xstep, xstep):
        yield (y,x) if steep else (x,y)
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax

def line100(*args):
    """ Bresenham line from (x0,y0) -> (x1,y1)

    with 100th precision (centimetric if you use SI).

    line100(1.21, 1.22, 1.23, 1.24)
    >> [[1.21, 1.22], [1.22, 1.23], [1.23, 1.24]]
    """
    return [[x/100., y/100.] for x,y in line(*[int(v*100.) for v in args])]
