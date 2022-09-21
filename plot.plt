set terminal png
set output "./figure/plot_y_xh.png"
set title "postion"
set xlabel "time [s]"
set ylabel "position [m]"
set grid
set key top right box linestyle 1 lt 2 lw 1
plot "./data.dat" u 1:2 lt rgbcolor 'red' title "obeserved", \
     "./data.dat" u 1:3 lt rgbcolor 'blue' ps 2 title "true", \
     "./data.dat" u 1:5 lt rgbcolor 'green' title "estimated"

set terminal png
set output "./figure/plot_xhd.png"
set title "velocity"
set xlabel "time [s]"
set ylabel "velocity [m/s]"
set grid
set key top right box linestyle 1 lt 2 lw 1
plot "./data.dat" u 1:4 lt rgbcolor 'blue' ps 2 title "true", \
     "./data.dat" u 1:6 lt rgbcolor 'green' title "estimated"

set terminal png
set output "./figure/plot_u.png"
set title "input force"
set xlabel "time [s]"
set ylabel "input force [N]"
set grid
set key top outside box linestyle 1 lt 2 lw 1
plot "./data.dat" u 1:7 lt rgbcolor 'blue' title "true"
