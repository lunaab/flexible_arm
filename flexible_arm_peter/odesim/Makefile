CFLAGS=-Wall -std=c++11 -I/usr/include/eigen3
LDFLAGS=-L/usr/local/lib -lode -ldrawstuff -lGL -lGLU -lX11 -pthread -lm -ltinyxml

all: explore sim

explore: explore.o Mesh.o
	g++ $(CFLAGS) -o explore.out explore.o Mesh.o $(LDFLAGS)

sim: sim.o Mesh.o
	g++ $(CFLAGS) -o sim.out sim.o Mesh.o $(LDFLAGS)

explore.o: explore.cpp Mesh.hpp
	g++ $(CFLAGS) -c explore.cpp

sim.o: sim.cpp Mesh.hpp
	g++ $(CFLAGS) -c sim.cpp

Mesh.o: Mesh.cpp Mesh.hpp
	g++ $(CFLAGS) -c Mesh.cpp

clean:
	rm -f explore.out sim.out explore.o sim.o Mesh.o
