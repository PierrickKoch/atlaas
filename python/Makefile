
atlaas/_atlaas*.so:
	@python setup.py build_ext --inplace

clean:
	@rm -rf atlaas/_*.so atlaas/_*.cpp build atlaas.*.tif out.* cloud.*

test: atlaas/_atlaas*.so
	@python test.py

install:
	@python setup.py install --user

colormaps.py:
	wget https://raw.githubusercontent.com/BIDS/colormap/master/colormaps.py
