#! /bin/sh
python pos2json.py
python movie2.py
(python catlaas.py && python patlaas.py && \
    avconv -i movie4atlaas.path.%5d.png -b 10M -an -c:v libx264 movie4atlaas.avi) &
(python cregion.py && python pregion.py && \
    avconv -i movie4region.path.%5d.png -b 8M -an -c:v libx264 movie4region.avi) &
