function [times, tot_times, total_times, nx, ny, nz, plane_count] = readCSV(path) 
  csv = csvread(path);
  num_runs = rows(csv) - 1;
  runs = 0:num_runs;
  times = zeros(num_runs, 8); % zeros(rows, cols)
  tot_times = zeros(num_runs, 4); 

  % Runtimes
  times(:,1) = csv(2:end,1) / 1000.0;  % downsample [ms]
  times(:,2) = csv(2:end,2) / 1000.0;  % build tree (filter) [ms]
  times(:,3) = csv(2:end,3) / 1000.0;  % search (filter) [ms]
  times(:,4) = csv(2:end,4) / 1000.0;  % delete (filter) [ms]
  times(:,5) = csv(2:end,5) / 1000.0;  % build tree (subcloud) [ms]
  times(:,6) = csv(2:end,6) / 1000.0;  % search (subcloud) [ms]
  times(:,7) = csv(2:end,7) / 1000.0;  % delete (subcloud) [ms]
  times(:,8) = csv(2:end,9) / 1000.0;  % plane + normal vector [ms]
  tot_times(:,1) = csv(2:end,1) / 1000.0;  % downsample [ms]
  tot_times(:,2) = times(:,2) + times(:,3) + times(:,4);
  tot_times(:,3) = times(:,5) + times(:,6) + times(:,7);
  tot_times(:,4) = csv(2:end,9) / 1000.0;  % plane + normal vector [ms]
  total_times = csv(2:end,10) / 1000.0; % TOTAL [ms]

  % Normal vector
  nx = csv(2:end,11); % nx
  ny = csv(2:end,12); % ny
  nz = csv(2:end,13); % nz
  
  % Plane Count
  plane_count = csv(2:end,14);
endfunction