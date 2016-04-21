import matplotlib.pyplot as plt

def hist(data, fsize=(12, 5)):
    plt.figure(figsize = fsize)
    plt.hist( data.flatten(), 100, log=1 )

def show(data, cmin=0, cmax=1, cmap='viridis', fsize=(14, 12), x=[], y=[],
         pad=None, fname=None, title=None):
    plt.figure(figsize = fsize)
    imgplot = plt.imshow( data, interpolation='none' )
    imgplot.set_clim(cmin, cmax)
    imgplot.set_cmap(cmap)
    plt.colorbar()
    if len(x) and len(y):
        plt.plot(x, y, 'r', scalex=False, scaley=False)
    if pad:
        plt.tight_layout(pad=pad)
    if title:
        plt.title(title)
    if fname:
        plt.savefig(fname)
