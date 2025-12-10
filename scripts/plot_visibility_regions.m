% Visualizes good/bad visibility regions based on roll and pitch angles
% 4 roll regions × 4 pitch regions = 8 unique zones

clear; close all;

[x, y, z] = sphere(100);

% Convert to spherical coordinates (roll = azimuth, pitch = elevation from xy-plane)
% roll: 0-360 deg 
% pitch: -90 to 90 deg
roll = atan2d(y, x);  % map points on sphere to -180 to 180 deg roll anlge using atan2 (deg) (forward / backward motion)
roll(roll < 0) = roll(roll < 0) + 360;  % Convert to 0-360 deg
pitch = asind(z);  % map points on sphere to -90 to 90 deg pitch angle using asin (deg)

% visibility function
% 4 pitch regions: around -90, -45, +45, +90 deg
% 4 roll regions: 0-90, 90-180, 180-270, 270-360 deg
visibility = zeros(size(roll));

for i = 1:size(roll, 1)
    for j = 1:size(roll, 2)
        r = roll(i, j);
        p = pitch(i, j);

        % Determine pitch region
        % 0°/180°:  VERY BAD - no ground visible (default horizontal orientatiion)
        % 90°/270°  MODERATE - sees ground close (small area but high point density)
        % ±45°:     GOOD - optimal ground view (large area)


        % TODO: Maybe change to 0.2 + 0.8 * diag for a more smooth transition between bad and moderate
        pitch_rad = p * pi/180; % convert to rad
        pitch_optim = abs(sin(2 * pitch_rad));  % period of 180° -> Max at ±45°, min at 0°/±90°; abs for symmetry -- e.g. sin(2*45°)=sin(90°)=1, sin(2*0°)=sin(0°)=0, sin(2*90°)=sin(180°)=0
        pitch_score = 0.1 + 0.9 * pitch_optim;  % 0.1 at 0°/±90°, 1.0 at ±45° -- avoids 0 visibility

        % Determine roll region
        % 0°/180°:              VERY BAD - no ground visible (default horizontal orientatiion)
        % 90°/270°:             MODERATE - sees ground but close (small area but high point density)
        % 45°/135°/225°/315°:   GOOD - optimal ground view (large area)
        roll_rad = r * pi/180; % convert to rad
        roll_optim = abs(sin(2 * roll_rad));  % period of 180° -> Max at 45°, 135°, 225°, 315°, min at 0°/180°; abs for symmetry -- e.g. sin(2*45°)=sin(90°)=1, sin(2*135°)=sin(270°)=1, sin(2*0°)=sin(0°)=0
        roll_score = 0.1 + 0.9 * roll_optim;  % 0.1 at 0°/180°, 1.0 at 45°, 135°, 225°, 315° -- avoids 0 visibility

        % Combined visibility score (0 = bad, 1 = good) 
        visibility(i, j) = pitch_score * roll_score; % TODO:currently only high when both are high -- maybe find better option
    end
end

% transitions between regions using Gaussian blur
vis_smooth = visibility;
kernel_size = 5; % size of neighborhood for avg
for i = 1:size(visibility, 1)
    for j = 1:size(visibility, 2)
        % Get neighborhood indices
        i_range = max(1, i-kernel_size):min(size(visibility,1), i+kernel_size); % avoid out-of-bounds
        j_range = max(1, j-kernel_size):min(size(visibility,2), j+kernel_size); % avoid out-of-bounds
        vis_smooth(i, j) = mean(mean(visibility(i_range, j_range))); % compute avg of defined neighborhood
    end
end

% Normalize
visibility = vis_smooth;
visibility = (visibility - min(visibility(:))) / (max(visibility(:)) - min(visibility(:))); % [0...1]

% Create figure
figure('Position', [100, 100, 1920, 1080]);

% Plot 1: Sphere with visibility coloring
subplot(1, 3, 1);
surf(x, y, z, visibility, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
colormap(jet);
cb = colorbar;
caxis([0, 1]);
ylabel(cb, '0=BAD, 1=GOOD', 'FontSize', 9, 'FontWeight', 'bold');
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Ground Plane Visibility Zones');
view(45, 30); % TODO: adjust view potentially but then also adjust annotations
% lighting gouraud;
% light('Position', [1, 1, 1]);
grid on;

% annotations for roll &pitch (maybe remove later)
text(1.4, 0, 0, 'Roll=0° (BAD)', 'HorizontalAlignment', 'left', 'FontSize', 8, 'FontWeight', 'bold');
text(0, -1.4, 0, 'Roll=270° (MOD)', 'HorizontalAlignment', 'center', 'FontSize', 8);
text(0, 0, 1.3, 'Pitch=+90° (BAD)', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
text(0, 0, -1.3, 'Pitch=-90° (BAD)', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');

% Plot 2: Frontal view for clarity of symmetry
subplot(1, 3, 2);
surf(x, y, z, visibility, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
colormap(jet);
cb = colorbar;
caxis([0, 1]);
ylabel(cb, '0=BAD, 1=GOOD', 'FontSize', 9, 'FontWeight', 'bold');
axis equal;
xlabel('X: Roll Axis'); 
zlabel('Z: Pitch Axis');
title('Front View: X-Z Plane (showing Roll & Pitch)');
view(0, 0);
% lighting gouraud;
% light('Position', [1, 1, 1]);
grid on;

% Plot 3: 2D map of visibility
subplot(1, 3, 3);
[roll_grid, pitch_grid] = meshgrid(0:2:360, -90:1:90);
vis_map = zeros(size(roll_grid));

for i = 1:size(roll_grid, 1)
    for j = 1:size(roll_grid, 2)
        r = roll_grid(i, j);
        p = pitch_grid(i, j);

        pitch_rad = p * pi/180;
        diagonal_pitch = abs(sin(2 * pitch_rad));
        pitch_score = 0.1 + 0.9 * diagonal_pitch;

        roll_rad = r * pi/180;
        diagonal_roll = abs(sin(2 * roll_rad));
        roll_score = 0.1 + 0.9 * diagonal_roll;

        vis_map(i, j) = pitch_score * roll_score;
    end
end

vis_map = (vis_map - min(vis_map(:))) / (max(vis_map(:)) - min(vis_map(:)));

imagesc(0:2:360, -90:1:90, vis_map);
colormap(jet);
cb = colorbar;
caxis([0, 1]);
ylabel(cb, '0=BAD, 1=GOOD', 'FontSize', 9, 'FontWeight', 'bold');
xlabel('Roll (degrees)');
ylabel('Pitch (degrees)');
title('Visibility Heat Map');
axis xy;
grid on;

% axes markings
set(gca, 'XTick', [0, 90, 180, 270, 360]);
set(gca, 'YTick', [-90, -45, 0, 45, 90]);

% % Add zone boundary lines
% hold on;
% % 4 roll regions: vertical lines at 0, 90, 180, 270
% for r = [0, 90, 180, 270, 360]
%     plot([r, r], [-90, 90], 'k-', 'LineWidth', 1.5);
% end
% % 4 pitch regions: horizontal lines
% for p = [-67.5, -22.5, 22.5, 67.5]
%     plot([0, 360], [p, p], 'k-', 'LineWidth', 1.5);
% end
% hold off;

fprintf('\n=== Visibility Zones ===\n');
fprintf('PITCH Regions:\n');
fprintf('  0°: VERY BAD (horizontal - no ground visible)\n');
fprintf('  ±90°: MODERATE (straight up/down - close ground, small area, high density)\n');
fprintf('  ±45°: GOOD (optimal tilt - large ground area visible)\n\n');
fprintf('ROLL Regions:\n');
fprintf('  0°/180°: VERY BAD (no ground visible - looking forward/back)\n');
fprintf('  90°/270°: MODERATE (close ground - small area, high point density)\n');
fprintf('  45°/135°/225°/315°: GOOD (optimal ground view - large area)\n\n');
fprintf('Color coding:\n');
fprintf('  Red (0.7-1.0): Good visibility\n');
fprintf('  Yellow (0.3-0.7): Moderate visibility\n');
fprintf('  Blue (0.0-0.3): Poor visibility\n');

function visibility_score = get_visibility(pitch_deg, roll_deg)
    pitch_rad = pitch_deg * pi/180;
    pitch_optim = abs(sin(2 * pitch_rad));
    pitch_score = 0.1 + 0.9 * pitch_optim;

    roll_rad = roll_deg * pi/180;
    roll_optim = abs(sin(2 * roll_rad));
    roll_score = 0.1 + 0.9 * roll_optim;

    visibility_score = pitch_score * roll_score;
end

fprintf('\n=== Example Queries ===\n');
test_angles = [0, 0; 45, 45; 90, 0; 45, 90; 0, 90];
for i = 1:size(test_angles, 1)
    p = test_angles(i, 1);
    r = test_angles(i, 2);
    vis = get_visibility(p, r);
    fprintf('Pitch=%3d°, Roll=%3d° -> Visibility = %.3f\n', p, r, vis);
end


