#!/bin/bash

rm data.dat
rm figure/*.png

g++ -c example/src/spring_mass_damper.cpp -o sp.o
g++ -c src/kalman_filter.cpp -o kf.o
g++ -c main.cpp -o main.o
g++ sp.o kf.o main.o -o main
rm *.o
./main

g++ -std=c++11 src/kalman_filter.cpp test/test_kalman_filter.cpp -o test_kalman_filter -L/usr/local/lib -lgtest -lgtest_main -lpthread
g++ -std=c++11 src/kalman_filter.cpp example/src/spring_mass_damper.cpp test/test_spring_mass_damper.cpp -o test_spring_mass_damper -L/usr/local/lib -lgtest -lgtest_main -lpthread
./test_kalman_filter
./test_spring_mass_damper

gnuplot plot.plt

eog ./figure/plot_y_xh.png &
eog ./figure/plot_xhd.png &
eog ./figure/plot_u.png &
