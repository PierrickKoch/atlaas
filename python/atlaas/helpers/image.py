import numpy
try:
    from PIL import Image
except ImportError:
    import Image # aka Pillow

def save(path, data, quality=90):
    if len(data.shape) == 3 and data.shape[2] == 2:
        # Pillow "Cannot handle this data type" mode=LA
        img = Image.fromarray(data[:,:,0])
        alp = Image.fromarray(data[:,:,1])
        img.putalpha(alp)
    else:
        img = Image.fromarray(data)
    # in case of JPEG or WebP, set quality to 90%, else this option is ignored
    img.save(path, quality=quality)

def load(path, dtype='uint8'):
    img = Image.open(path)
    arr = numpy.array(img.getdata(), dtype)
    return arr.reshape(img.size[1], img.size[0], arr.shape[1])

def zero(path):
    save(path, load(path) * 0)
