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


## Content
* include

		#include <armadillo>
		using namespace arma;

* declaration

		double observe_Z=0.0;
		double estimate_Z=0.0;

		mat X(2,1);mat X_(2,1);
		mat P(2,2);mat P_(2,2);
		mat F(2,2);mat F_TRAN(2,2);
		mat Q(2,2);mat FP(2,2);mat FPF_TRAN(2,2);
		mat H(1,2);mat HP_(1,2);mat H_TRAN(2,1);mat P_H_TRAN(2,1);mat HP_H_TRAN(1,1);mat HP_H_TRAN_ADD_R(1,1);
		mat HX_(1,1);
		mat K(2,1);mat K_TERM(2,1);mat KH(2,2);
		mat P_TERM(2,2);
		mat R(1,1);
		mat I(2,2);

* Kalman core

		void KalmanFilter(){
		    X_=F*X;
		    X_.print("X_=F*X: ");
		    P_=F*P*F.t()+Q;
		    P_.print("P_=F*P*F'+Q: ");
		    K=(P_*H.t())/as_scalar(H*P_*H.t()+R);
		    K.print("K: ");
		    X=X_+K*(observe_Z-as_scalar(H*X_));
		    X.print("X: ");
		    estimate_Z=X(0,0);
		  }
* Initialize parameter

		   X<<0<<endr<<0<<endr;
		   P<<1<<0<<endr<<0<<1<<endr;
		   F<<1<<0.5<<endr<<0<<1<<endr;
		   Q<<0.00001<<0<<endr<<0<<0.00001<<endr;
		   H<<1<<0<<endr;
		   R<<0.1<<endr;
		   I<<1<<0<<endr<<0<<1<<endr;

* use kalman
   	
		KalmanFilter();

