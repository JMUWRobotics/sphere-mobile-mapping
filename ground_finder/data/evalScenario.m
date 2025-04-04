function [] = evalScenario(filename, max_entries, gt)
  printf("--------- EVALUATION ---------\n");
  printf("File: %s\n", filename);
  
  plot_times_mat = zeros(6, 5, 8); # zeros(# filter-subcloud options, # of plane alg, elems per stack)
  plot_tot_times_mat = zeros(6, 5, 4); # same dimension
  plot_total_times = zeros(6, 5); # zeros(# filter-subcloud options, # of plane alg)
  fail_rates = zeros(6, 5);
  mean_abs_angle_error = zeros(6, 5);
  variances = zeros(6, 5);

  % Filter-Subcloud 1: none-geo - Order: LSF, PCA, RAN, RHT, RHT2 (for all)
  files_sc1 = [["lsf/none-geo-", filename, ".csv"]; ["pca/none-geo-", filename, ".csv"]; ["ran/none-geo-", filename, ".csv"]; ["rht/none-geo-", filename, ".csv"]; ["rht2/none-geo-", filename, ".csv"]];
  files_sc1 = cellstr(files_sc1);

  % Filter-Subcloud 2: geo-geo
  files_sc2 = [["lsf/geo-geo-", filename, ".csv"]; ["pca/geo-geo-", filename, ".csv"]; ["ran/geo-geo-", filename, ".csv"]; ["rht/geo-geo-", filename, ".csv"]; ["rht2/geo-geo-", filename, ".csv"]];
  files_sc2 = cellstr(files_sc2);

  % Filter-Subcloud 3: geo-kdt
  files_sc3 = [["lsf/geo-kdt-", filename, ".csv"]; ["pca/geo-kdt-", filename, ".csv"]; ["ran/geo-kdt-", filename, ".csv"]; ["rht/geo-kdt-", filename, ".csv"]; ["rht2/geo-kdt-", filename, ".csv"]];
  files_sc3 = cellstr(files_sc3);

  % Filter-Subcloud 4: none-kdt
  files_sc4 = [["lsf/none-kdt-", filename, ".csv"]; ["pca/none-kdt-", filename, ".csv"]; ["ran/none-kdt-", filename, ".csv"]; ["rht/none-kdt-", filename, ".csv"]; ["rht2/none-kdt-", filename, ".csv"]];
  files_sc4 = cellstr(files_sc4);

  % Filter-Subcloud 5: kdt-kdt
  files_sc5 = [["lsf/kdt-kdt-", filename, ".csv"]; ["pca/kdt-kdt-", filename, ".csv"]; ["ran/kdt-kdt-", filename, ".csv"]; ["rht/kdt-kdt-", filename, ".csv"]; ["rht2/kdt-kdt-", filename, ".csv"]];
  files_sc5 = cellstr(files_sc5);

  % Filter-Subcloud 6: kdt-geo
  files_sc6 = [["lsf/kdt-geo-", filename, ".csv"]; ["pca/kdt-geo-", filename, ".csv"]; ["ran/kdt-geo-", filename, ".csv"]; ["rht/kdt-geo-", filename, ".csv"]; ["rht2/kdt-geo-", filename, ".csv"]];
  files_sc6 = cellstr(files_sc6);
  
  files = [files_sc1, files_sc4, files_sc6, files_sc2, files_sc3, files_sc5];

  abs_err_angles = zeros(5, max_entries);
  fail_counts = zeros(5, 1);
  
  % Label for Filter-Subcloud in time plots
  labels_text = [["none-geo"]; ["none-kdt"]; ["kdt-geo"]; ["geo-geo"]; ["geo-kdt"];  ["kdt-kdt"]];
  labels_text = cellstr(labels_text);

  for j=1:6
    printf("-- Case: %s\n", char(labels_text(j, 1)));
    for i=1:5
      % Read file 
      [times, tot_times, total_time, nx, ny, nz, plane_count] = readCSV(char(files(i, j)));
      
      % Calculate avg. times [ms]
      avg_times = takeMeanTimes(times);
      avg_tot_times = takeMeanTimes(tot_times);
      avg_total_time = takeMeanTimes(total_time);
      plot_times_mat(j,i,:) = avg_times;
      plot_tot_times_mat(j,i,:) = avg_tot_times;
      plot_total_times(j, i) = avg_total_time;
      
      % Calculate mean absolute angle error and plot angle over iterations [ms]
      index = max_entries - length(nx) + 1;
      [mean_abs_angle_error(j,i), abs_err_angles(i,index:end), fail_counts(i)] = absAngleError(nx, ny, nz, gt, plane_count);
      variances(j,i) = stdVariance(nx, ny, nz, gt, plane_count, mean_abs_angle_error(j,i));
    endfor
    figure(j);
    plotError(abs_err_angles);
    abs_err_angles = zeros(5, max_entries);
    title(char(labels_text(j, 1)));
    
    fail_rates(j,:) = fail_counts / max_entries * 100.0; # [%]
    
    # Output
    disp("Fail counts: LSF-PCA-RAN-RHT-RHT2");
    disp(fail_counts);
    disp("");
  endfor
  %
  
  % ------------- write to text file for latex  -------------  
  ## Time file
  fid = fopen(["results_tables/", filename, "_times.txt"], 'w+');
  
  text_lab1 = [["& \\textbf{Downsample [ms]} & "]; ["& \\textbf{Build Tree [ms]} & "]; ["& \\textbf{Radius Search [ms]} & "]; ["& \\textbf{Delete [ms]} & "];
          ["& \\textbf{Build Tree [ms]} & "]; ["& \\textbf{Cheese Search [ms]} & "]; ["& \\textbf{Delete [ms]} & "]; ["& \\textbf{Plane [ms]} & "]];
  text_lab1 = cellstr(text_lab1);
  
  text_lab2 = [["& \\textbf{Downsample [ms]} & "]; ["& \\textbf{Build Tree [ms]} & "]; ["& \\textbf{Radius Search [ms]} & "]; ["& \\textbf{Delete [ms]} & "];
          ["& \\textbf{Build Tree [ms]} & "]; ["& \\textbf{k-\\gls{nns} [ms]} & "]; ["& \\textbf{Delete [ms]} & "]; ["& \\textbf{Plane [ms]} & "]];
  text_lab2 = cellstr(text_lab2);
  
  text_lab = [text_lab1, text_lab2, text_lab1, text_lab1, text_lab2, text_lab2];
  
  text_tab = [["\\multirow{5}{*}{\\rotatebox[origin=c]{90}{none-geo}}"]; ["\\multirow{6}{*}{\\rotatebox[origin=c]{90}{none-kdt}}"];
              ["\\multirow{9}{*}{\\rotatebox[origin=c]{90}{kdt-geo}}"];  ["\\multirow{6}{*}{\\rotatebox[origin=c]{90}{geo-geo}}"];
              ["\\multirow{9}{*}{\\rotatebox[origin=c]{90}{geo-kdt}}"];  ["\\multirow{10}{*}{\\rotatebox[origin=c]{90}{kdt-kdt}}"]];
  text_tab = cellstr(text_tab);
  
  for j=1:6
    fprintf(fid, '%s ', char(text_tab(j)));
    fprintf(fid, '\n');
    for k=1:8
      fprintf(fid, '%s ', char(text_lab(k,j)));
      avg_tmp = 0.0;
      for i=1:5
        fprintf(fid, '%.3f & ', plot_times_mat(j,i,k));
        avg_tmp = avg_tmp + plot_times_mat(j,i,k);
      endfor
      avg_tmp = avg_tmp / 5;
      fprintf(fid, "\\textbf{%.3f} \\\\*", avg_tmp);
      avg_tmp = 0.0;
      fprintf(fid, '\n');
    endfor
    % 
    fprintf(fid, "\\cline{2-8}");
    fprintf(fid, '\n');
    fprintf(fid, "& \\textbf{Total [ms]} & ");
    for i=1:5
      fprintf(fid, '%.3f & ', plot_total_times(j,i));
      avg_tmp = avg_tmp + plot_total_times(j,i);
    endfor
    avg_tmp = avg_tmp / 5;
    fprintf(fid, "\\textbf{%.3f} \\\\*", avg_tmp);
    avg_tmp = 0.0;
    fprintf(fid, '\n');
    fprintf(fid, "& \\textbf{Fail Rate [perc]} & ");
    for i=1:5
      fprintf(fid, '%.3f & ', fail_rates(j,i));
      avg_tmp = avg_tmp + fail_rates(j,i);
    endfor
    avg_tmp = avg_tmp / 5;
    fprintf(fid, "\\textbf{%.3f} \\\\", avg_tmp);
    avg_tmp = 0.0;
    fprintf(fid, '\n');
    fprintf(fid, "\\hline");
    fprintf(fid, '\n');
  endfor
  fclose(fid);
  
  ## Mean Angle error and variance file
  fid = fopen(["results_tables/", filename, "_error.txt"], 'w+');
  text_lab = [["& \\textbf{\\gls{mae} [deg]} & "]; ["& \\textbf{Variance [deg$^2$]} & "]];
  text_lab = cellstr(text_lab);
  
  text_tab = [["\\multirow{2}{*}{none-geo}"]; ["\\multirow{2}{*}{none-kdt}"];
              ["\\multirow{2}{*}{kdt-geo}"];  ["\\multirow{2}{*}{geo-geo}"];
              ["\\multirow{2}{*}{geo-kdt}"];  ["\\multirow{2}{*}{kdt-kdt}"]];
  text_tab = cellstr(text_tab);

  for j=1:6
    fprintf(fid, '%s ', char(text_tab(j)));
    fprintf(fid, '\n');
    # Angle Error
    fprintf(fid, '%s ', char(text_lab(1)));
    for i=1:5
      fprintf(fid, '%.3f', mean_abs_angle_error(j,i));
      if i != 5
        fprintf(fid, ' & ');
      else 
        fprintf(fid, ' \\\\');
      endif
    endfor
    fprintf(fid, '\n');
    % 
    # Variance
    fprintf(fid, '%s ', char(text_lab(2)));
    for i=1:5
      fprintf(fid, '%.3f', variances(j,i));
      if i != 5
        fprintf(fid, ' & ');
      else 
        fprintf(fid, ' \\\\');
      endif
    endfor
    fprintf(fid, '\n');
    fprintf(fid, "\\hline");
    fprintf(fid, '\n');
  endfor
  fclose(fid);
  

  % ------------- time plots  -------------  
  % Label for Filter-Subcloud in time plots
  labels = {"none-geo", "none-kdt", "kdt-geo", "geo-geo", "geo-kdt",  "kdt-kdt"};
  
  figure(101);
  plotBarStackGroups(plot_times_mat, labels);
  plot_name = ["results_plots/", filename, "_times_full"];
  print(plot_name, "-depslatex"); 
  
  figure(102);
  plotBarStackGroups(plot_tot_times_mat, labels);
  plot_name = ["results_plots/", filename, "_times_reduced"];
  print(plot_name, "-depslatex"); 

endfunction
%