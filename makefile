all: main.o
	g++47 -o main.exe main.o -L/usr/local/lib64/ -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab -Wl,-rpath,/usr/local/lib64/ && ./main.exe

main.o: main.cpp
	g++47 -c -o main.o main.cpp -std=c++11 -O3 -Wall -Wextra -Werror -I. -I/usr/local/include/ -I/usr/local/include/boost_1.53.0/

clean:
	rm -f main.o main.exe