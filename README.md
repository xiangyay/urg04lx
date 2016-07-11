# HOKUYO URG04LX sensor appliaction in RASPBERRY 3
# We use this sensor to detecte the outline of the game props. 
# Contact me: yuxiangyang326@gmail.com

HOW TO INSTALL THIS LIB
	1. send all file to RASPBERRY 3
	2. cd urg04lx
	   make
	   sudo make install

The default installation directory on linux is /usr/local, on the subdirectories lib/, bin/ and bin/ respectively. Change the PREFIX variable on /Makefile if you need changing the installation location for these directories.
	For example, to change installation directory to /usr/, edit /Makefile as follows:
		PREFIX = /usr 

HOW TO COMPILE A PROGRAM WITH THIS LIB
	Compile command example for test.c file.
		% gcc -I/usr/local/include/urg_c test.c -o test -lurg_c -lm
	You can also use the urg_c-config compile script (installed with the library). The example above can be compiled as:
		% gcc `urg_c-config --cflags` test.c -o test `urg_c-config --libs` -lm