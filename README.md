ATLAAS
======

*Atlaas, the Digital Terrain Modelization library*

[![Build Status](https://travis-ci.org/pierriko/atlaas.png?branch=master)]
(https://travis-ci.org/pierriko/atlaas)

* http://gdal.org
* http://www.openrobots.org/wiki
* http://trac.laas.fr/git/atlaas

[![youtube](https://i2.ytimg.com/vi/k1-6gbYnmMU/sddefault.jpg "youtube")](http://youtube.com/embed/k1-6gbYnmMU?rel=0)


INSTALL
-------

    git clone http://trac.laas.fr/git/atlaas && cd atlaas
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/devel ..
    make -j8 && make install


CONTRIBUTE
----------

Code is available on GitHub at https://github.com/pierriko/atlaas

Feel free to fork, pull request or submit issues to improve the project!

* https://github.com/pierriko/atlaas/fork
* https://github.com/pierriko/atlaas/issues
* https://github.com/pierriko/atlaas/pulls
* https://help.github.com/articles/fork-a-repo
* https://help.github.com/articles/using-pull-requests

### STYLE

Please configure your editor to insert 4 spaces instead of TABs, maximum line
length to 79, `lower_case_with_underscores` instead of `CamelCase`. Most of the
rules are taken from [Python PEP8](http://www.python.org/dev/peps/pep-0008/)

Other ideas can be found in Google Guides:
[Python](http://google-styleguide.googlecode.com/svn/trunk/pyguide.html),
[C++](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml).


LICENSE
-------

[BSD 2-Clause](http://opensource.org/licenses/BSD-2-Clause)

Copyright © 2013-2014 CNRS-LAAS
