%% Evaluate final avg runtimes and mae over all experiments

files = [["results_tables/draussen_hoch_times.txt"];
         ["results_tables/draussen_runter_times.txt"];
         ["results_tables/flur_times.txt"];
         ["results_tables/physik_fast_times.txt"];
         ["results_tables/physik_slow_times.txt"];
         ["results_tables/saal_times.txt"];
         ["results_tables/tunnel_times.txt"];
         ["results_tables/wand_times.txt"]];
files = cellstr(files);

files2 = [["results_tables/draussen_hoch_error.txt"];
         ["results_tables/draussen_runter_error.txt"];
         ["results_tables/flur_error.txt"];
         ["results_tables/physik_fast_error.txt"];
         ["results_tables/physik_slow_error.txt"];
         ["results_tables/saal_error.txt"];
         ["results_tables/tunnel_error.txt"];
         ["results_tables/wand_error.txt"]];
files2 = cellstr(files2);

index_plane = 7*5;
index_fail_rate = 9*5;
index_pp = 8*5;
index_mae = 0;

runtime_avg_plane = zeros(6,5); % LSF, PCA, RAN, RHT, RHT2
fail_rate_avg = zeros(6,5);
runtime_total_avg = zeros(6,5); # preprocessing
mae_avg = zeros(6,5);

results = zeros(300, 8);
results2 = zeros(60, 8);
for i=1:8
  a = textread(char(files(i)), "%f");
  b = textread(char(files2(i)), "%f");
  results(:,i) = rmmissing(a);
  results2(:,i) = rmmissing(b);
endfor
%

for i=1:6
  for m=1:8
    for j=1:5
      runtime_avg_plane(i,j) = runtime_avg_plane(i,j) + results(index_plane + j, m);
      fail_rate_avg(i,j) = fail_rate_avg(i,j) + results(index_fail_rate + j, m);
      runtime_total_avg(i,j) = runtime_total_avg(i,j) + results(index_pp + j, m);
      mae_avg(i,j) = mae_avg(i,j) + results2(index_mae + j, m);
    endfor
  endfor
  index_plane = index_plane + 10*5;
  index_fail_rate = index_fail_rate + 10*5;
  index_pp = index_pp + 10*5;
  index_mae = index_mae + 10;
endfor

% Take mean and display
runtime_avg_plane = runtime_avg_plane / 8.0;
fail_rate_avg = fail_rate_avg / 8.0;

mae_avg = mae_avg / 8.0;
##for i=1:6
##  for j=1:5
##    fprintf('%.3f ', mae_avg(i,j));
##  endfor
##  disp("");
##endfor
%

runtime_total_avg = runtime_total_avg / 8.0;
runtime_pp_avg = runtime_total_avg - runtime_avg_plane;
runtime_avg_total = zeros(6,1);
runtime_avg_pp = zeros(6,1);
for i=1:6
  sum = 0.0;
  sum2 = 0.0;
  for j=1:5
    sum = sum + runtime_total_avg(i,j);
    sum2 = sum2 + runtime_pp_avg(i,j);
  endfor
  runtime_avg_pp(i) = sum2 / 5.0;
  runtime_avg_total(i) = sum / 5.0;
endfor

for i=1:6
    fprintf('%.3f ', runtime_avg_pp(i));
endfor
disp("");
for i=1:6
    fprintf('%.3f ', runtime_avg_total(i));
endfor
disp("");
%


