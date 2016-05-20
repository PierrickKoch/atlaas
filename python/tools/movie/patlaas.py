import os
import json
from PIL import Image, ImageDraw
from atlaas.helpers.gdal2 import gdal2

with open('poses.json') as f:
    poses = json.load(f)

g = gdal2('merged.atlaas.00000.tif')
xy_atlaas = [tuple(map(int, g.c2p(x,y))) for x,y in poses]

inc = 0
while os.path.isfile('movie4atlaas.%05i.png'%inc):
    im = Image.open('movie4atlaas.%05i.png'%inc)
    dr = ImageDraw.Draw(im)
    dr.line(xy_atlaas[:inc+1], fill=(255,0,0,255)) # , width=2)
    im.save('movie4atlaas.path.%05i.png'%inc)
    inc += 1
