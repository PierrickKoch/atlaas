atlaas/python
=============

```

# build and test atlaas python binding
make test
# cleanup test data
rm atlaas.*.tif pcl.*.pcd
# run morse-ros simulation
source /opt/ros/indigo/setup.bash
morse run -g 800x600 test_morse.py

# run ros-client
python test_ros.py

# run WebGL visualization tool
ln -s atlaas.jpg /tmp/
# git clone git://github.com/pierriko/threeviz.git -b atlaas
cd ~/work/threeviz
xdg-open http://localhost:8686
python -m SimpleHTTPServer 8686
# press [Space] in browser

```

