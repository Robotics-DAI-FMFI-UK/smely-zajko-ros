all:	demo

demo:
	@echo "compiling C program..."
	gcc -o rgb2hsv.o -c rgb2hsv.c -g
	@echo "linking C library..."
	gcc -shared -Wl,-soname,librgb2hsv.so.1 -o librgb2hsv.so.1.0.0 rgb2hsv.o -lc -g
	@echo "creating library symbolic links..."
	ln -sf librgb2hsv.so.1.0.0 librgb2hsv.so.1
	ln -sf librgb2hsv.so.1.0.0 librgb2hsv.so
	@echo "running example"
	python use_rgb2hsv.py
