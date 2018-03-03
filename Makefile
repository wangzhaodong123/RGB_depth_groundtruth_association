all:rgb_dep_gro_associate

rgb_dep_gro_associate:rgb_dep_gro_associate.o
	g++ -std=c++11 -O3 -Wno-deprecated -o rgb_dep_gro_associate rgb_dep_gro_associate.o
	rm -f *.o
rgb_dep_gro_associate.o: src/rgb_dep_gro_associate.cpp
	g++ -std=c++11 -O3 -Wno-deprecated -c src/rgb_dep_gro_associate.cpp
clean:
	rm -f *.o rgb_dep_gro_associate
