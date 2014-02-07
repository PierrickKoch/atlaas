TODO
====

break everything
----------------

*07-02-2014 after benchmark with `valgrind --tool=callgrind atlaas`*

![callgrind](http://homepages.laas.fr/pkoch/wm/callgrind.png "callgrind")

- divide atlaas in 2 class
  - 1 map manager -> {9->25} maplets of 256x256 px (standard)
  - speed up and simplify slide_to / sub_{load,save}
  - externalize export8u, save all
  - let another node ``gdal_merge.py atlaas.*x*.tif`` for display (no critic)
  - use a map<index, cell_info> for ground-swap instead of vector?
- use standard convention for maplets names ``$PWD/$zoom/$x/$y.tif``
  - http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Tile_servers

