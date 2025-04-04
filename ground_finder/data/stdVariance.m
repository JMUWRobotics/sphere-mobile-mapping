function [std_var] = stdVariance(nx, ny, nz, gt_angles, plane_count, mean_ang_error)
  sum_dif_angle = 0.0;
  count = 0;
  down = [0.0, 0.0, -1.0];

  
  for i = 1:length(nx)
    % Check if invalid vector (skip call if yes)
    if nx(i) != -1 && ny(i) != -1 && nz(i) != -1
      v = [nx(i), ny(i), nz(i)];
      
      ## NOTE: taking difference between gt angle zu down vector and measured angle zu down,
      ## wegen yaw drift; unknown rotation around z-achse! uns geht es nur um die steigung!
      gt_angle = gt_angles(plane_count(i)+1); # [deg]
      measure_angle = acos(dot(v, down)) * 180 / pi; # Angle between measured n-vector and down [deg]
      angle = gt_angle - measure_angle; 
      
      dif_angle = (angle - mean_ang_error) * (angle - mean_ang_error); # [deg²]
      sum_dif_angle = sum_dif_angle + dif_angle;
      count = count + 1;
    endif    
  endfor
  std_var = sum_dif_angle / (count - 1)
endfunction
