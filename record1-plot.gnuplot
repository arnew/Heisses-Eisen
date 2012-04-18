
set datafile separator ','
set xlabel 'reference temperature [°C]'
set ylabel 'thermocouple voltage [V]'
set format y "%01.3E"
plot 	 "./record1-raw.csv" using ($1*2.56/1024/5e-3):($2*2.56/1024/400) every ::::1700 with points title 'measurement 1 (heating)', \
"./record1-raw.csv" using ($1*2.56/1024/5e-3):($2*2.56/1024/400) every ::1730 with points title 'measurement 1 (cooling)' 
