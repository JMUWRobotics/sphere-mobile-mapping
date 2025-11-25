function [mean] = takeMeanTimes(times_mat)
  mean = zeros(1, columns(times_mat));
  count = 0;
  for i = 1:rows(times_mat)
    # Check if run was a fail if yes take out
    if times_mat(i, columns(times_mat)) != -1 # TODO update to -1!
      mean = mean + times_mat(i,:);
      count = count+1;
    endif
  endfor  
  mean = mean / count;
endfunction