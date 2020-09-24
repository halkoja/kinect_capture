# Path to libfreenect2:
LFR2=

capture: capt.cpp
	g++ -std=c++11 -o capt -I${LFR2}/include capt.cpp -Wl,-rpath=${LFR2}/lib -L${LFR2}/lib -lfreenect2
