
atlaas*.so:
	@python3.4 setup.py build_ext --inplace

clean:
	@rm -rf atlaas*.so atlaas.cpp build atlaas.*.tif out.*

test: atlaas*.so
	@python3.4 test.py

transformations.py:
	wget -cq http://www.lfd.uci.edu/~gohlke/code/transformations.py
