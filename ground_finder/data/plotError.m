function [] = plotError(angles, ran = 0)
  green = [34, 182, 125];
  green = green / 255;
  blue = [34, 113, 183];
  blue = blue / 255;
  red = [183, 34, 49];
  red = red / 255;
  yellow = [220, 195, 34];
  yellow = yellow / 255;
  magenta = [204, 35, 173];
  magenta = magenta / 255;

  if ran==0
    % Plot abs angle error
    plot(angles(1,:), 'r');
    hold on;
    plot(angles(2,:), 'color', yellow);
    hold on;
    plot(angles(3,:), 'color', green);
    hold on;
    plot(angles(4,:), 'b');
    hold on;
    plot(angles(5,:), 'color', magenta);

    xlabel("Iterations");
    ylabel("Abs. Angle Error [°]");
    legend("LSF", "PCA", "RANSAC", "RHT", "RHT2");
  else
    % Plot abs angle error
    plot(angles(1,:), 'r');
    hold on;
    plot(angles(2,:), 'b');

    xlabel("Iterations");
    ylabel("Abs. Angle Error [°]");
    legend("RAN-PCA", "RAN-LSF");
  endif

endfunction
