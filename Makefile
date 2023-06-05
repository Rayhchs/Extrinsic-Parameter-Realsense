CC=g++
CFLAGS=-c -Wall
INCLUDE = -I /usr/include/eigen3
INCLUDE += -I/usr/local/include/opencv4
LDFLAGS += -L/usr/local/lib `pkg-config --libs opencv`
LDFLAGS += -lyaml-cpp

SOURCES=main.cpp lib/utils.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -o $@ $(OBJECTS) $(LDFLAGS) 

.cpp.o:
	$(CC) $< -o $@ $(CFLAGS) $(INCLUDE)

clean:
	rm -rf *o $(EXECUTABLE)