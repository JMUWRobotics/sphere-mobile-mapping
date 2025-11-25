function [mean_ang_error, angles, fail_count] = absAngleError(nx, ny, nz, gt_angles, plane_count)
  sum_abs_angle = 0.0;
  count = 0;
  fail_count = 0;
  down = [0.0, 0.0, -1.0];
  
  angles = zeros(1, length(nx));
  prev_angle = 0.0;
  
  for i = 1:length(nx)
    % Check if invalid vector (skip call if yes)
    if nx(i) != -1 && ny(i) != -1 && nz(i) != -1
      v = [nx(i), ny(i), nz(i)];
      
      ## NOTE: taking difference between gt angle zu down vector and measured angle zu down,
      ## wegen yaw drift; unknown rotation around z-achse! uns geht es nur um die steigung!
      gt_angle = gt_angles(plane_count(i)+1); # [deg]
      measure_angle = acos(dot(v, down)) * 180 / pi; # Angle between measured n-vector and down [deg]
      angle = abs(gt_angle - measure_angle); # TODO switch to rad?
      
      angles(i) = angle;
      prev_angle = angle;
      sum_abs_angle = sum_abs_angle + angle;
      count = count + 1;
    else
      angles(i) = prev_angle;
      fail_count = fail_count + 1;
    endif    
  endfor
  mean_ang_error = sum_abs_angle/count
 
endfunction
