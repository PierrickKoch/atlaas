import os
import json
from PIL import Image, ImageDraw
from atlaas.helpers.gdal2 import gdal2

with open('poses.json') as f:
    poses = json.load(f)

g = gdal2('merged.region.00000.png')
xy_region = [tuple(map(int, g.c2p(x,y))) for x,y in poses]

inc = 0
while os.path.isfile('movie4region.%05i.png'%inc):
    im = Image.open('movie4region.%05i.png'%inc)
    dr = ImageDraw.Draw(im)
    dr.line(xy_region[:inc+1], fill=(255,0,0,255))
    im.save('movie4region.path.%05i.png'%inc)
    inc += 1
