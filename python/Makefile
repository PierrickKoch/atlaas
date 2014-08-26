
atlaas*.so:
	@python3.4 setup.py build_ext --inplace

clean:
	@rm -rf atlaas*.so atlaas.cpp build atlaas.*.tif out.* __pycache__

test: atlaas*.so
	@python3.4 test.py

py2:
	@python setup.py build_ext --inplace
