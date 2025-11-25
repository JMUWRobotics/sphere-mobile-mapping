function [] = evalRansac(max_entries_arr, gt_arr)
  printf("--------- EVALUATION RANSAC ---------\n");

  plot_times_mat = zeros(8, 2, 8); # zeros(# files, # of plane alg, elems per stack)
  plot_tot_times_mat = zeros(8, 2, 4); # same dimension
  plot_total_times = zeros(8, 2); # zeros(# filter-subcloud options, # of plane alg)
  fail_rates = zeros(8, 2);
  mean_abs_angle_error = zeros(8, 2);
  variances = zeros(8, 2);

  % Plane RANSAC with LSF - Order: Flur, Saal, Wand, Physik_slow, Physik_fast, draussen_hoch, draussen_runter, tunnel
  files_lsf = [["ran/geo-geo-lsf_flur.csv"]; ["ran/geo-geo-lsf_saal.csv"]; ["ran/geo-geo-lsf_wand.csv"]; ["ran/geo-geo-lsf_physik_slow.csv"];
               ["ran/geo-geo-lsf_physik_fast.csv"]; ["ran/geo-geo-lsf_draussen_hoch.csv"]; ["ran/geo-geo-lsf_draussen_runter.csv"];
               ["ran/geo-geo-lsf_tunnel.csv"]];
  files_lsf = cellstr(files_lsf);

  % Plane RANSAC with PCA - Order: Flur, Saal, Wand, Physik_slow, Physik_fast, draussen_hoch, draussen_runter, tunnel
  files_pca = [["ran/geo-geo-flur.csv"]; ["ran/geo-geo-saal.csv"]; ["ran/geo-geo-wand.csv"]; ["ran/geo-geo-physik_slow.csv"];
               ["ran/geo-geo-physik_fast.csv"]; ["ran/geo-geo-draussen_hoch.csv"]; ["ran/geo-geo-draussen_runter.csv"];
               ["ran/geo-geo-tunnel.csv"]];
  files_pca = cellstr(files_pca);

  files = [files_pca, files_lsf];
  fail_counts = zeros(2, 1);

  % Label for Filter-Subcloud in time plots
  labels_text = [["A"]; ["B"]; ["C"]; ["D"]; ["E"]; ["F"]; ["G"]; ["H"]];
  labels_text = cellstr(labels_text);

  for j=1:8
    printf("-- Case: %s\n", char(labels_text(j, 1)));
    abs_err_angles = zeros(2, max_entries_arr(j));
    for i=1:2
      % Read file
      [times, tot_times, total_time, nx, ny, nz, plane_count] = readCSV(char(files(j, i)));

      % Calculate avg. times [ms]
      avg_times = takeMeanTimes(times);
      avg_tot_times = takeMeanTimes(tot_times);
      avg_total_time = takeMeanTimes(total_time);
      plot_times_mat(j,i,:) = avg_times;
      plot_tot_times_mat(j,i,:) = avg_tot_times;
      plot_total_times(j, i) = avg_total_time;

      % Calculate mean absolute angle error and plot angle over iterations [ms]
      index = max_entries_arr(j) - length(nx) + 1;
      [mean_abs_angle_error(j,i), abs_err_angles(i,index:end), fail_counts(i)] = absAngleError(nx, ny, nz, gt_arr(j,:), plane_count);
      variances(j,i) = stdVariance(nx, ny, nz, gt_arr(j,:), plane_count, mean_abs_angle_error(j,i));
    endfor
    figure(j);
    plotError(abs_err_angles, 1.0);
    abs_err_angles = zeros(5, max_entries_arr(j));
    title(char(labels_text(j, 1)));

    fail_rates(j,:) = fail_counts / max_entries_arr(j) * 100.0; # [%]

    # Output
    disp("Fail counts: RAN-PCA - RAN-LSF");
    disp(fail_counts);
    disp("");
  endfor
  %

  % ------------- write to text file for latex  -------------
  ## Time file
  fid = fopen(["results_tables/ransac_comp_times.txt"], 'w+');

  text_lab = [["& \\textbf{Prepr. [ms]} & "]; ["& \\textbf{Plane [ms]} & "]; ["& \\textbf{Total [ms]} & "];
              ["& \\textbf{Fail Rate [perc]} & "]; ["& \\textbf{\\gls{mae} [deg]} & "]; ["& \\textbf{Variance [deg$^2$]} & "]];
  text_lab = cellstr(text_lab);

  text_tab = [["\\multirow{6}{*}{\\rotatebox[origin=c]{90}{RAN-PCA}}"]; ["\\multirow{6}{*}{\\rotatebox[origin=c]{90}{RAN-LSF}}"]];
  text_tab = cellstr(text_tab);

  for j=1:2
    fprintf(fid, '%s ', char(text_tab(j)));
    fprintf(fid, '\n');
    for k=1:6
      fprintf(fid, '%s ', char(text_lab(k)));
      for i=1:8
        if k == 1
          tmp = plot_tot_times_mat(i,j,1) + plot_tot_times_mat(i,j,2) + plot_tot_times_mat(i,j,3);
          fprintf(fid, '%.3f', tmp);
        elseif k == 2
          fprintf(fid, '%.3f', plot_tot_times_mat(i,j,4));
        elseif k == 3
          fprintf(fid, '%.3f', plot_total_times(i,j));
        elseif k == 4
          fprintf(fid, '%.3f', fail_rates(i,j));
        elseif k == 5
          fprintf(fid, '%.3f', mean_abs_angle_error(i,j));
        elseif k == 6
          fprintf(fid, '%.3f', variances(i,j));
        endif

        if i != 8
          fprintf(fid, ' & ');
        endif
      endfor
      fprintf(fid, ' \\\\ \n');
      if k == 2
        fprintf(fid, "\\cline{2-10}");
        fprintf(fid, '\n');
      endif
    endfor
    fprintf(fid, ' \\hline \n');
  endfor
  fclose(fid);


  % ------------- time plots  -------------
  % Label for Filter-Subcloud in time plots
  labels = {"A", "B", "C", "D", "E", "F", "G", "H"};

  figure(101);
  plotBarStackGroups(plot_times_mat, labels, 1.0);
##  plot_name = ["results_plots/", filename, "_times_full"];
##  print(plot_name, "-depslatex");

  figure(102);
  plotBarStackGroups(plot_tot_times_mat, labels, 1.0);
##  plot_name = ["results_plots/", filename, "_times_reduced"];
##  print(plot_name, "-depslatex");

endfunction
%
