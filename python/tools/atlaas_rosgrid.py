#! /usr/bin/env python
"""
Atlaas to OccupancyGrid node
The idea is to publish 2 grids:
- one resulting of dumped tiles (GeoTiff)
- one from the current tiles in memory (via export8u)

NOTE those are not OccupancyGrid, just distribution of elevation (debug only)
"""
import os
import atlaas
from atlaas.helpers.rosgrid import atlaas8u_grid, atlaas_grid
import rospy
from nav_msgs.msg import OccupancyGrid

mtime = 0
jpg_pose = None
rospy.init_node("atlaas_grid")
atlaas_path = rospy.get_param("atlaas_path", os.environ.get("ATLAAS_PATH", "."))
filepath = os.path.join(atlaas_path, "atlaas.jpg")
mergedpath = os.path.join(atlaas_path, "out.tif")
pub = rospy.Publisher("/grid", OccupancyGrid, queue_size=5, latch=True)
pub_glob = rospy.Publisher("/grid_glob", OccupancyGrid, queue_size=5, latch=True)

while not rospy.is_shutdown():
    rospy.sleep(0.1)
    if not pub.get_num_connections() and not pub_glob.get_num_connections():
        continue # skip if no client
    if not os.path.isfile(filepath):
        continue
    filestat = os.stat(filepath)
    if filestat.st_mtime == mtime:
        continue # skip if not modified
    mtime = filestat.st_mtime
    grid = atlaas8u_grid(filepath, stamp=rospy.Time.now())
    pose = grid.info.origin.position.x, grid.info.origin.position.y
    pub.publish(grid)
    if not jpg_pose or pose != jpg_pose:
        jpg_pose = pose
        # if we did atlaas::slide (the internal tiles in memory moved)
        atlaas.merge(os.path.join(atlaas_path, "atlaas.*x*.tif"), mergedpath)
        if not os.path.isfile(mergedpath):
            continue # skip if no tiles on disk
        pub_glob.publish(atlaas_grid(mergedpath, stamp=rospy.Time.now()))
