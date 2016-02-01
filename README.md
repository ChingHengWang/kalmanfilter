# kalmanfilter

## Armadillo package setup
* download http://arma.sourceforge.net/download.html   Stable Version->armadillo-6.500.4.tar.gz

		tar xvf armadillo-6.400.4.tar.gz 

		cd armadillo-6.400.4.tar.gz 

		cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr .

		make

		sudo make install

## Add line in cmake file

* find_package(Armadillo)
* 	-- ZACH : Armadillo_DIR: /usr/share/Armadillo/CMake
* 	-- ZACH : ARMADILLO_INCLUDE_DIR: /usr/include
*	-- ZACH : ARMADILLO_LIBRARIES: /usr/lib/libarmadillo.so
* include_directories( ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIR})
* add_executable (example src/example.cpp)
* target_link_libraries( example ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

		catkin_make
