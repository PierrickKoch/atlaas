
atlaas*.so:
	@python setup.py build_ext --inplace

clean:
	@rm -rf atlaas*.so atlaas.cpp build atlaas.*.tif out.* __pycache__

test: atlaas*.so
	@python test.py

install:
	@python setup.py install --user
