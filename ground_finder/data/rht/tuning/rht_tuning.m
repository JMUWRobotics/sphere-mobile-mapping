clear;
disp("");

% file 1 - rhoNum = 50, phiNum = 90, thetaNum = 180, rhoMax = 5, accumulatorMax = 10;
csv1 = csvread("geo-kdt-k01.csv");
num_runs1 = rows(csv1) - 1;
runs1 = 0:num_runs1;
nx1 = csv1(:,11); % nx
ny1 = csv1(:,12); % ny
nz1 = csv1(:,13); % nz
tp1 = csv1(:,9) / 1000;  % plane time [ns]
tp1_avg = sum(tp1) / num_runs1

for i = 1:num_runs1
  if(nz1(i) == -1)
    nz1(i) = 1;
  endif
endfor

% file 2 - rhoNum = 10, phiNum = 90, thetaNum = 180, rhoMax = 5, accumulatorMax = 10;
csv2 = csvread("geo-kdt-k02.csv");
num_runs2 = rows(csv2) - 1;
runs2 = 0:num_runs2;
nx2 = csv2(:,11); % nx
ny2 = csv2(:,12); % ny
nz2 = csv2(:,13); % nz
tp2 = csv2(:,9) / 1000;  % plane time [ns]
tp2_avg = sum(tp2) / num_runs2

for i = 1:num_runs2
  if(nz2(i) == -1)
    nz2(i) = 1;
  endif
endfor

% file 3 - rhoNum = 2, phiNum = 90, thetaNum = 180, rhoMax = 5, accumulatorMax = 10;
csv3 = csvread("geo-kdt-k03.csv");
num_runs3 = rows(csv3) - 1;
runs3 = 0:num_runs3;
nx3 = csv3(:,11); % nx
ny3 = csv3(:,12); % ny
nz3 = csv3(:,13); % nz
tp3 = csv3(:,9) / 1000;  % plane time [ns]
tp3_avg = sum(tp3) / num_runs3

for i = 1:num_runs3
  if(nz3(i) == -1)
    nz3(i) = 1;
  endif
endfor

% file 4 - rhoNum = 25, phiNum = 90, thetaNum = 180, rhoMax = 5, accumulatorMax = 10;
csv4 = csvread("geo-kdt-k04.csv");
num_runs4 = rows(csv4) - 1;
runs4 = 0:num_runs4;
nx4 = csv4(:,11); % nx
ny4 = csv4(:,12); % ny
nz4 = csv4(:,13); % nz
tp4 = csv4(:,9) / 1000;  % plane time [ns]
tp4_avg = sum(tp4) / num_runs4

for i = 1:num_runs4
  if(nz4(i) == -1)
    nz4(i) = 1;
  endif
endfor


##csv5 = csvread("geo-kdt-test01.csv");
##num_runs5 = rows(csv5) - 1;
##runs5 = 0:num_runs5;
##nx5 = csv5(:,11); % nx
##ny5 = csv5(:,12); % ny
##nz5 = csv5(:,13); % nz
##tp5 = csv5(:,9) / 1000;  % plane time [ns]
##tp5_avg = sum(tp5) / num_runs5
##
##for i = 1:num_runs5
##  if(nz5(i) == -1)
##    nz5(i) = 1;
##  endif
##endfor
##
##
##csv6 = csvread("geo-kdt-tune017.csv");
##num_runs6 = rows(csv6) - 1;
##runs6 = 0:num_runs6;
##nx6 = csv6(:,11); % nx
##ny6 = csv6(:,12); % ny
##nz6 = csv6(:,13); % nz
##tp6 = csv6(:,9) / 1000;  % plane time [ns]
##tp6_avg = sum(tp6) / num_runs6
##
##for i = 1:num_runs6
##  if(nz6(i) == -1)
##    nz6(i) = 1;
##  endif
##endfor

figure("101");
plot(runs1, nz1, "r", runs2, nz2, "g", runs3, nz3, "b", runs4, nz4, "k"); #, runs5, nz5, "m", runs6, nz6, "c");

