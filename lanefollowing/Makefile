TARGET      = lanefollowing

DIR_DXL    = /usr/local/lib
DIR_OBJS   = .objects

CX          = g++
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g

INCLUDES   = -I/home/ncslab/share/DynamixelSDK/c/include/dynamixel_sdk
LIBRARIES  += -ldxl_x86_c
LIBRARIES  += -lrt
LIBRARIES  += `pkg-config opencv4 --cflags --libs`

SOURCES = lanefollowing.cpp

.PHONY: all clean

$(TARGET):  $(SOURCES)
	$(CX) $(CXFLAGS) $(SOURCES) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo


