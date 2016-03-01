import matplotlib.pyplot as plt

def hist(data, fsize=(12, 5)):
    plt.figure(figsize = fsize)
    plt.hist( data.flatten(), 100, log=1 )

def show(data, cmin=0, cmax=1, cmap='viridis', fsize=(14, 12), x=[], y=[]):
    plt.figure(figsize = fsize)
    imgplot = plt.imshow( data, interpolation='none' )
    imgplot.set_clim(cmin, cmax)
    imgplot.set_cmap(cmap)
    plt.colorbar()
    if len(x) and len(y):
        plt.plot(x, y, 'r', scalex=False, scaley=False)
    plt.tight_layout(pad=0)
