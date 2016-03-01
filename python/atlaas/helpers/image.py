import Image # aka PIL
import numpy

def save(path, data, quality=90):
    # in case of JPEG or WebP, set quality to 90%, else this option is ignored
    Image.fromarray(data).save(path, quality=quality)

def load(path, dtype='uint8'):
    img = Image.open(path)
    arr = numpy.array(img.getdata(), dtype)
    return arr.reshape(img.size[1], img.size[0], arr.shape[1])
