#! /usr/bin/env python
import os
import gdal
import atlaas
from matplotlib import cm
import cv2

class TrackbarData:
    def __init__(self, val=0):
        self.value = val
    def create(self, name='Trackbar', window='Window'):
        cv2.createTrackbar(name, window, int(self.value), 10, self.update)
    def update(self, val):
        self.value = val

T1 = TrackbarData(0)
T2 = TrackbarData(10)
cv2.namedWindow('image')
T1.create('T1', 'image')
T2.create('T2', 'image')

def convert(fin, cmin, cmax, cmap=cm.spectral):
    geo = gdal.Open(fin)
    img = geo.ReadAsArray() # get band as a numpy.array
    img = (img - cmin) * (1./(cmax - cmin))
    img[img > 1] = 1
    img[img < 0] = 0
    return (cmap(img)*255).astype('uint8')

pcd_id = 0
patern = 'cloud.%05i.pcd'
ftif = 'atlaas.zmean.tif'

test = atlaas.Atlaas()
test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)

while os.path.isfile(patern%pcd_id):
    test.merge_file(patern%pcd_id)
    test.export_zmean(ftif)
    image = convert(ftif, T1.value, T2.value)
    if not pcd_id:
        fourcc = cv2.cv.CV_FOURCC(*'FMP4')
        size = (image.shape[1], image.shape[0]) # tuple(reversed(image.shape))
        video = cv2.VideoWriter('atlaas.zmean.avi', fourcc, 100, size)
    video.write( image )
    cv2.imshow('image', image)
    pcd_id += 1
    if cv2.waitKey(1) & 0xFF == 27: # aka ESCAPE
        break

video.release()
