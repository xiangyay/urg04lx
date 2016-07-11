URG_C_LIB_STATIC = liburg_c.a
URG_C_LIB_SHARED = $(shell if test `echo $(OS) | grep Windows`; then echo "urg_c.dll"; else echo "liburg_c.so"; fi)

TARGET = $(URG_C_LIB_STATIC)\
		 $(URG_C_LIB_SHARED)

OBJ_C = urg_sensor.o urg_utils.o urg_debug.o urg_connection.o \
        urg_ring_buffer.o urg_serial.o urg_serial_utils.o urg_tcpclient.o

include ../build_rule.mk

CFLAGS = -g -O2 $(INCLUDES) -I../include -fPIC
LDLIBS = -lm $(shell if test `echo $(OS) | grep Windows`; then echo "-lwsock32 -lsetupapi"; fi)

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET)

$(URG_C_LIB_STATIC) : $(OBJ_C)
	$(AR) rv $@ $(OBJ_C)

$(URG_C_LIB_SHARED) : $(OBJ_C)
	$(CC) $(CFLAGS) -shared -o $@ $(OBJ_C) $(LDLIBS)

urg_serial_utils.o : urg_serial_utils_linux.c
