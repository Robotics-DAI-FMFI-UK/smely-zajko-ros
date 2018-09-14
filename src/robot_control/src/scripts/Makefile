all:	librgb2hsv.so 

librgb2hsv.so: rgb2hsv.c evalhsv.c
	gcc -o rgb2hsv.o -c rgb2hsv.c -g
	gcc -o evalhsv.o -c evalhsv.c -g
	gcc -shared -Wl,-soname,librgb2hsv.so.1 -o librgb2hsv.so.1.0.0 rgb2hsv.o evalhsv.o -lc -g
	ln -sf librgb2hsv.so.1.0.0 librgb2hsv.so.1
	ln -sf librgb2hsv.so.1.0.0 librgb2hsv.so

