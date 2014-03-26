TODO
====

- use <tr1/array> instead of <array> for boost fallback on no C++11 systems.
- benchmark benefit of libtiff over gdal for real-time application.
- rename atlaas in libatlaas to avoid conflict with genom's atlaas.pc

- https://github.com/ethz-asl/libpointmatcher
- http://pointclouds.org/documentation/tutorials/#registration-tutorial
- consider points < 20m as clean, merge live stream p3d
  for the others, filter points < 20m and do ICP
- atlaas.cpp:311 (atlaas::merge) : add field number of swap to classify
  if a cell is stable or not and it first seen / first swap time
- atlaas.cpp:43  (atlaas::merge) : add I/O for playback (rosbag like)
- add thread + deque for slide:tile_save, std::copy in main, gdal::save in T

- let another node ``gdal_merge.py atlaas.*x*.tif`` for display (no critic)
- use standard convention for maplets names ``$PWD/$zoom/$x/$y.tif``
  - http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Tile_servers

