# Makefile for polyload test
# - Uriah
#

DEP = ../../dependencies
GLFW_TOP = $(DEP)/glfw-2.7.4
LIBGLFW = $(GLFW_TOP)/lib/cocoa/libglfw.a
VC_TOP = $(DEP)/vcollide-2.01


# Compiler settings
CC = g++
CFLAGS = -O2 -g

INCLUDES = -I$(GLFW_TOP)/include -I$(VC_TOP)/include 

# Linker settings
LIBS = $(LIBGLFW) -lVCollide -lRAPID -lm  
LFLAGS = -L$(VC_TOP)/lib -L$(VC_TOP)/RAPID $(LIBS) -framework Cocoa -framework OpenGL


OBJS = polytope.o polyObject.o graphics.o main.o
TARGET = polyload

all:	polyload


$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(LFLAGS) -o $@ $(OBJS)  $(LIBS)

.c.o: 
	$(CC) $(CFLAGS) $(INCLUDES) -c $<

clean:
	rm -f $(OBJS) $(TARGET)
