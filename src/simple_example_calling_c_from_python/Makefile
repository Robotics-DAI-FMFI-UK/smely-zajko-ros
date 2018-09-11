all:	demo

demo:
	@echo "compiling C program..."
	gcc -o simple.o -c simple.c
	@echo "linking C library..."
	gcc -shared -Wl,-soname,libsimple.so.1 -o libsimple.so.1.0.0 simple.o -lc
	@echo "creating library symbolic links..."
	ln -sf libsimple.so.1.0.0 libsimple.so.1
	ln -sf libsimple.so.1.0.0 libsimple.so
	@echo "running example"
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.
	python use_simple.py
