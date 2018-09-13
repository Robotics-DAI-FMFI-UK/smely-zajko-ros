all:	librgb2hsv.so libanalyse.so

librgb2hsv.so:
	@echo "compiling C program..."
	gcc -o rgb2hsv.o -c rgb2hsv.c -g
	@echo "linking C library..."
	gcc -shared -Wl,-soname,librgb2hsv.so.1 -o librgb2hsv.so.1.0.0 rgb2hsv.o -lc -g
	@echo "creating library symbolic links..."
	ln -sf librgb2hsv.so.1.0.0 librgb2hsv.so.1
	ln -sf librgb2hsv.so.1.0.0 librgb2hsv.so
	@echo "running example"
	python use_rgb2hsv.py

libanalyse.so:
	@echo "compiling C program..."
	gcc -o analyse.o -c analyse.c -g -fPIC
	@echo "linking C library..."
	gcc -shared -Wl,-soname,libanalyse.so.1 -o libanalyse.so.1.0.0 analyse.o -lc -g -fPIC
	@echo "creating library symbolic links..."
	ln -sf libanalyse.so.1.0.0 libanalyse.so.1
	ln -sf libanalyse.so.1.0.0 libanalyse.so
	@echo "running example"
	python analyse.py
