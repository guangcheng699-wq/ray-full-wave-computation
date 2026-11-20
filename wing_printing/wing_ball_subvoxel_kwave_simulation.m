% Butterfly ball centers -> incremental occupancy -> medium maps

clc; clear;
script_dir = fileparts(mfilename('fullpath'));
% close all;

run_model = 2;                  % 1 = CPU (kspaceFirstOrder3D)
                                % 2 = GPU (kspaceFirstOrder3DG)
phase_mode = 'path_integral';
num_segments = 1000000;
cfl          = 0.3;
t_end        = 0.6e-3;          % [s]

ball_data_mode = 'mat';   % 'mat' or 'csv'
ball_mat_file = fullfile(script_dir, 'butterfly_outer_contour_results.mat');
ball_csv_file = fullfile(script_dir, 'wing_ball_numbering.csv');
sphere_diameter_mm_override = [];
num_balls_to_apply = 50;
ball_unit_scale = 1e-3;
ball_center_target = [0, 0, -5e-3];
ball_rot_xyz_deg   = [0, 0, 0];

c0   = 346;       % [m/s]
rho0 = 1.2;       % [kg/m^3]
c_obj   = 500;    % [m/s]
rho_obj = 25;     % [kg/m^3]

dx = 0.5e-3;
dy = 0.5e-3;
dz = 0.5e-3;
Nx = 401;
Ny = 401;
Nz = 341;

bbox_pad_vox = 2;
ss_xyz       = [5, 5, 5];

% 'union_prob' : 1 - (1-a)(1-b)
% 'max'        : max(a,b)
occupancy_merge_mode = 'union_prob';
material_map_mode = 'fractional';
binary_threshold  = 0.5;

show_ball_before_after = true;
show_occupancy_slices  = true;
show_medium_slices     = true;
show_mask_isosurface   = true;
show_ball_centers_plot = true;
show_medium_3d_overview = true;
medium3d_slice_alpha    = 0.22;
medium3d_marker_size    = 85;
show_focus_in_medium3d  = true;
use_occ_centroid_slice  = true;
custom_slice_idx_xyz    = [];

f0 = 40e3;                    % [Hz]
input_pressure = 1;           % [Pa]
Nx_elem = 16;
Ny_elem = 16;
num_elem_single = Nx_elem * Ny_elem;
num_elem_total  = 2 * num_elem_single;
elem_diam = 9e-3;             % [m]
pitch     = 10e-3;            % [m]
focus_pos = [0, 0, 1] * 1e-3; % [m]
z_bottom  = -84e-3;           % [m]
z_top     = +84e-3;           % [m]
array_gap = z_top - z_bottom; % [m]
x0 = 0;
y0 = 0;

lambda = c0 / f0;
fprintf('波长 lambda = %.3f mm\n', lambda * 1e3);
fprintf('pitch / lambda = %.3f\n', pitch / lambda);

if ~ismember(lower(ball_data_mode), {'mat','csv'})
    error('ball_data_mode 只能取 ''mat'' 或 ''csv''。');
end

if strcmpi(ball_data_mode, 'mat')
    if exist(ball_mat_file, 'file') ~= 2
        error('未找到 MAT 文件: %s', ball_mat_file);
    end
else
    if exist(ball_csv_file, 'file') ~= 2
        error('未找到 CSV 文件: %s', ball_csv_file);
    end
    if isempty(sphere_diameter_mm_override)
        error('当 ball_data_mode = ''csv'' 时，必须给出 sphere_diameter_mm_override。');
    end
end

if numel(ss_xyz) ~= 3 || any(ss_xyz < 1) || any(mod(ss_xyz,1) ~= 0)
    error('ss_xyz 必须是形如 [nx ny nz] 的正整数三元组。');
end

if ~ismember(lower(material_map_mode), {'fractional','binary_threshold'})
    error('material_map_mode 只能取 ''fractional'' 或 ''binary_threshold''。');
end

if ~ismember(lower(occupancy_merge_mode), {'union_prob','max'})
    error('occupancy_merge_mode 只能取 ''union_prob'' 或 ''max''。');
end

kgrid = kWaveGrid(Nx, dx, Ny, dy, Nz, dz);
x_vec = kgrid.x_vec;
y_vec = kgrid.y_vec;
z_vec = kgrid.z_vec;

fprintf('网格范围 x: [%.2f, %.2f] mm\n', min(x_vec)*1e3, max(x_vec)*1e3);
fprintf('网格范围 y: [%.2f, %.2f] mm\n', min(y_vec)*1e3, max(y_vec)*1e3);
fprintf('网格范围 z: [%.2f, %.2f] mm\n', min(z_vec)*1e3, max(z_vec)*1e3);

grid_pts = double(Nx) * double(Ny) * double(Nz);
fprintf('总网格点数 = %.0f\n', grid_pts);
fprintf('单个 full-size single 3D 数组约占 = %.2f MiB\n', grid_pts * 4 / 1024^2);

medium.sound_speed = c0   * ones(Nx, Ny, Nz, 'single');
medium.density     = rho0 * ones(Nx, Ny, Nz, 'single');

fprintf('\n===== 读取球心数据 =====\n');

[ball_table_all, sphere_diameter_m] = loadButterflyBallData( ...
    ball_data_mode, ball_mat_file, ball_csv_file, sphere_diameter_mm_override);

sphere_radius_m = sphere_diameter_m / 2;

fprintf('读取到总球数 = %d\n', height(ball_table_all));
fprintf('球直径 = %.3f mm\n', sphere_diameter_m * 1e3);

centers_raw_m = [ball_table_all.x_mm, ball_table_all.y_mm, ball_table_all.z_mm] * ball_unit_scale;
[centers_tfm_m, centers_bbox_center_raw] = transformBallCenters( ...
    centers_raw_m, ball_center_target, ball_rot_xyz_deg);

num_total_balls = size(centers_tfm_m, 1);
num_balls_use   = max(0, min(num_total_balls, num_balls_to_apply));
selected_ids    = 1:num_balls_use;
centers_use_m   = centers_tfm_m(selected_ids, :);

fprintf('本次用于构建非均匀介质的球数 = %d\n', num_balls_use);

ball_bbox_min = min(centers_tfm_m - sphere_radius_m, [], 1);
ball_bbox_max = max(centers_tfm_m + sphere_radius_m, [], 1);

fprintf('变换后球堆整体包围盒 [mm]:\n');
fprintf('  x: [%.3f, %.3f]\n', ball_bbox_min(1)*1e3, ball_bbox_max(1)*1e3);
fprintf('  y: [%.3f, %.3f]\n', ball_bbox_min(2)*1e3, ball_bbox_max(2)*1e3);
fprintf('  z: [%.3f, %.3f]\n', ball_bbox_min(3)*1e3, ball_bbox_max(3)*1e3);

if show_ball_before_after
    figure('Name', 'Ball centers before/after transform');
    tiledlayout(1,2);

    nexttile;
    scatter3(centers_raw_m(:,1)*1e3, centers_raw_m(:,2)*1e3, centers_raw_m(:,3)*1e3, 15, 'filled');
    axis equal; grid on;
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title('Original ball centers');

    nexttile;
    scatter3(centers_tfm_m(:,1)*1e3, centers_tfm_m(:,2)*1e3, centers_tfm_m(:,3)*1e3, 15, 'filled');
    hold on;
    plot3(ball_center_target(1)*1e3, ball_center_target(2)*1e3, ball_center_target(3)*1e3, ...
        'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
    axis equal; grid on;
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title('Transformed ball centers');
end

if show_ball_centers_plot && num_balls_use > 0
    figure('Name', 'Selected ball centers');
    scatter3(centers_use_m(:,1)*1e3, centers_use_m(:,2)*1e3, centers_use_m(:,3)*1e3, 18, 'filled');
    axis equal; grid on;
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title(sprintf('Selected first %d ball centers', num_balls_use));
    view(3);
end

if num_balls_use > 0
    bbox_min = min(centers_use_m - sphere_radius_m, [], 1);
    bbox_max = max(centers_use_m + sphere_radius_m, [], 1);

    if bbox_min(1) < min(x_vec) || bbox_max(1) > max(x_vec) || ...
       bbox_min(2) < min(y_vec) || bbox_max(2) > max(y_vec) || ...
       bbox_min(3) < min(z_vec) || bbox_max(3) > max(z_vec)
        error('变换后的球堆超出当前网格范围，请调整 ball_center_target / ball_rot_xyz_deg / 网格。');
    end
end

fprintf('\n===== 基于球心的增量 occupancy 构建 =====\n');
fprintf('亚体素采样 = [%d, %d, %d]，每个粗体素共 %d 个子点\n', ...
    ss_xyz(1), ss_xyz(2), ss_xyz(3), prod(ss_xyz));

occ_full = zeros(Nx, Ny, Nz, 'single');

for k = 1:num_balls_use
    c_ball = centers_use_m(k, :);

    [bbox_ix, bbox_iy, bbox_iz] = getSphereLocalBBoxIndices( ...
        c_ball, sphere_radius_m, x_vec, y_vec, z_vec, dx, dy, dz, bbox_pad_vox);

    x_loc = x_vec(bbox_ix);
    y_loc = y_vec(bbox_iy);
    z_loc = z_vec(bbox_iz);

    occ_ball_local = computeSingleSphereOccFraction( ...
        c_ball, sphere_radius_m, x_loc, y_loc, z_loc, dx, dy, dz, ss_xyz);

    occ_old_local = occ_full(bbox_ix, bbox_iy, bbox_iz);

    switch lower(occupancy_merge_mode)
        case 'union_prob'
            occ_new_local = 1 - (1 - occ_old_local) .* (1 - occ_ball_local);
        case 'max'
            occ_new_local = max(occ_old_local, occ_ball_local);
    end

    occ_full(bbox_ix, bbox_iy, bbox_iz) = single(occ_new_local);

    if mod(k, max(1, floor(max(num_balls_use,1)/10))) == 0 || k == num_balls_use
        fprintf('已累计写入球数: %d / %d\n', k, num_balls_use);
    end
end

num_occ = nnz(occ_full > 0);
fprintf('占据率 > 0 的粗体素数 = %d\n', num_occ);

if num_occ == 0
    warning('球心体素化后没有任何占据体素，请检查球径、单位或位置。');
end

switch lower(material_map_mode)
    case 'fractional'
        medium.sound_speed = single(c0   + occ_full .* (c_obj   - c0));
        medium.density     = single(rho0 + occ_full .* (rho_obj - rho0));
        fprintf('材料映射模式 = fractional（推荐）\n');

    case 'binary_threshold'
        mask_full = occ_full >= binary_threshold;
        medium.sound_speed(mask_full) = single(c_obj);
        medium.density(mask_full)     = single(rho_obj);
        fprintf('材料映射模式 = binary_threshold, 阈值 = %.3f\n', binary_threshold);
end

fprintf('已将球堆 occupancy 写入 medium.sound_speed / medium.density。\n');

occ_sum = sum(double(occ_full(:)));

if occ_sum > 0
    x_mass = squeeze(sum(sum(double(occ_full), 2), 3));
    y_mass = squeeze(sum(sum(double(occ_full), 1), 3));
    z_mass = squeeze(sum(sum(double(occ_full), 1), 2));
    xc_occ = sum(x_vec(:) .* x_mass(:)) / occ_sum;
    yc_occ = sum(y_vec(:) .* y_mass(:)) / occ_sum;
    zc_occ = sum(z_vec(:) .* z_mass(:)) / occ_sum;
else
    xc_occ = ball_center_target(1);
    yc_occ = ball_center_target(2);
    zc_occ = ball_center_target(3);
end

[~, ix_occ] = min(abs(x_vec - xc_occ));
[~, iy_occ] = min(abs(y_vec - yc_occ));
[~, iz_occ] = min(abs(z_vec - zc_occ));

fprintf('occupancy 质心 [mm] = [%.3f, %.3f, %.3f]\n', ...
    xc_occ*1e3, yc_occ*1e3, zc_occ*1e3);

if show_occupancy_slices
    xy_occ = squeeze(occ_full(:,:,iz_occ));
    xz_occ = squeeze(occ_full(:,iy_occ,:));
    yz_occ = squeeze(occ_full(ix_occ,:,:));

    figure('Name', 'Occupancy fraction slices from ball centers');
    tiledlayout(1,3);

    nexttile;
    imagesc(y_vec*1e3, x_vec*1e3, xy_occ);
    axis image; colorbar; colormap(parula); clim([0 1]);
    xlabel('y [mm]'); ylabel('x [mm]');
    title(sprintf('XY occupancy at z = %.2f mm', z_vec(iz_occ)*1e3));

    nexttile;
    imagesc(z_vec*1e3, x_vec*1e3, xz_occ);
    axis image; colorbar; colormap(parula); clim([0 1]);
    xlabel('z [mm]'); ylabel('x [mm]');
    title(sprintf('XZ occupancy at y = %.2f mm', y_vec(iy_occ)*1e3));

    nexttile;
    imagesc(z_vec*1e3, y_vec*1e3, yz_occ);
    axis image; colorbar; colormap(parula); clim([0 1]);
    xlabel('z [mm]'); ylabel('y [mm]');
    title(sprintf('YZ occupancy at x = %.2f mm', x_vec(ix_occ)*1e3));
end

if show_medium_slices
    figure('Name', 'Sound speed slices from ball-center medium');
    tiledlayout(1,3);

    nexttile;
    imagesc(y_vec*1e3, x_vec*1e3, squeeze(medium.sound_speed(:,:,iz_occ)));
    axis image; colorbar; colormap(jet);
    xlabel('y [mm]'); ylabel('x [mm]');
    title(sprintf('XY slice at z = %.2f mm', z_vec(iz_occ)*1e3));

    nexttile;
    imagesc(z_vec*1e3, x_vec*1e3, squeeze(medium.sound_speed(:,iy_occ,:)));
    axis image; colorbar; colormap(jet);
    xlabel('z [mm]'); ylabel('x [mm]');
    title(sprintf('XZ slice at y = %.2f mm', y_vec(iy_occ)*1e3));

    nexttile;
    imagesc(z_vec*1e3, y_vec*1e3, squeeze(medium.sound_speed(ix_occ,:,:)));
    axis image; colorbar; colormap(jet);
    xlabel('z [mm]'); ylabel('y [mm]');
    title(sprintf('YZ slice at x = %.2f mm', x_vec(ix_occ)*1e3));
end

if show_mask_isosurface && num_occ > 0
    occ_vis = permute(occ_full, [2 1 3]);

    figure('Name', 'Ball-stack occupancy isosurface');
    p = patch(isosurface( ...
        x_vec*1e3, y_vec*1e3, z_vec*1e3, occ_vis, 0.5));

    isonormals(x_vec*1e3, y_vec*1e3, z_vec*1e3, occ_vis, p);

    set(p, 'FaceColor', [0.2 0.8 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    hold on;

    if num_balls_use > 0
        plot3(centers_use_m(:,1)*1e3, centers_use_m(:,2)*1e3, centers_use_m(:,3)*1e3, ...
            'k.', 'MarkerSize', 8);
    end

    plot3(xc_occ*1e3, yc_occ*1e3, zc_occ*1e3, ...
        'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');

    axis equal; grid on; camlight; lighting gouraud;
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title(sprintf('Ball-stack occupancy isosurface (0.5), N = %d', num_balls_use));
    view(3);
end

c_ref = double(max(medium.sound_speed(:)));
[t_array, dt] = makeTimeCFL(dx, dy, dz, c_ref, cfl, t_end);
kgrid.setTime(numel(t_array), dt);

Nt = numel(kgrid.t_array);
fprintf('dt = %.3e s, Nt = %d\n', dt, Nt);

x_centers = ((0:Nx_elem-1) - (Nx_elem-1)/2) * pitch + x0;
y_centers = ((0:Ny_elem-1) - (Ny_elem-1)/2) * pitch + y0;
elem_pos = zeros(num_elem_total, 3);
bottom_idx = 1:num_elem_single;
top_idx    = num_elem_single + 1 : num_elem_total;
karray = kWaveArray;
idx = 0;
for iy = 1:Ny_elem
    for ix = 1:Nx_elem
        idx = idx + 1;
        pos = [x_centers(ix), y_centers(iy), z_bottom];
        elem_pos(idx, :) = pos;
        face_pos = pos + [0, 0, +1e-3];
        karray.addDiscElement(pos, elem_diam, face_pos);
    end
end

idx2 = num_elem_single;
for iy = 1:Ny_elem
    for ix = 1:Nx_elem
        idx2 = idx2 + 1;
        pos = [x_centers(ix), y_centers(iy), z_top];
        elem_pos(idx2, :) = pos;
        face_pos = pos + [0, 0, -1e-3];
        karray.addDiscElement(pos, elem_diam, face_pos);
    end
end

elem_pos_bottom = elem_pos(bottom_idx, :);
elem_pos_top    = elem_pos(top_idx, :);
elem_r = elem_diam / 2;

if min(elem_pos(:,1) - elem_r) < min(x_vec) || max(elem_pos(:,1) + elem_r) > max(x_vec)
    error('阵列在 x 方向超出网格，请增大 Nx 或 dx。');
end
if min(elem_pos(:,2) - elem_r) < min(y_vec) || max(elem_pos(:,2) + elem_r) > max(y_vec)
    error('阵列在 y 方向超出网格，请增大 Ny 或 dy。');
end
if min(elem_pos(:,3)) < min(z_vec) || max(elem_pos(:,3)) > max(z_vec)
    error('阵列在 z 方向超出网格，请增大 Nz 或 dz。');
end
if focus_pos(1) < min(x_vec) || focus_pos(1) > max(x_vec) || ...
   focus_pos(2) < min(y_vec) || focus_pos(2) > max(y_vec) || ...
   focus_pos(3) < min(z_vec) || focus_pos(3) > max(z_vec)
    error('focus_pos 超出网格范围。');
end

aperture_x = (Nx_elem - 1) * pitch + elem_diam;
aperture_y = (Ny_elem - 1) * pitch + elem_diam;

fprintf('\n===== 对置双阵列结构 =====\n');
fprintf('单阵列阵元数: %d\n', num_elem_single);
fprintf('总阵元数: %d\n', num_elem_total);
fprintf('阵列孔径: %.1f mm × %.1f mm\n', aperture_x*1e3, aperture_y*1e3);
fprintf('阵元直径: %.1f mm\n', elem_diam*1e3);
fprintf('阵元中心距: %.1f mm\n', pitch*1e3);
fprintf('上下阵列间距: %.1f mm\n', array_gap*1e3);
fprintf('下阵列 z = %.1f mm, 朝 +z\n', z_bottom*1e3);
fprintf('上阵列 z = %.1f mm, 朝 -z\n', z_top*1e3);
fprintf('焦点位置 = [%.1f, %.1f, %.1f] mm\n', focus_pos(1)*1e3, focus_pos(2)*1e3, focus_pos(3)*1e3);

figure('Name', 'Opposed dual 16x16 planar arrays');
hold on; grid on; axis equal;

plot3(elem_pos_bottom(:,1)*1e3, elem_pos_bottom(:,2)*1e3, elem_pos_bottom(:,3)*1e3, ...
    'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Bottom array');
plot3(elem_pos_top(:,1)*1e3, elem_pos_top(:,2)*1e3, elem_pos_top(:,3)*1e3, ...
    'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Top array');
plot3(focus_pos(1)*1e3, focus_pos(2)*1e3, focus_pos(3)*1e3, ...
    'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'DisplayName', 'Focus');
plot3(ball_center_target(1)*1e3, ball_center_target(2)*1e3, ball_center_target(3)*1e3, ...
    'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Ball target center');

xlabel('x [mm]');
ylabel('y [mm]');
zlabel('z [mm]');
title('Opposed dual 16x16 planar arrays + focus + ball stack');
legend('Location', 'best');
view(3);

if show_medium_3d_overview
    if use_occ_centroid_slice || isempty(custom_slice_idx_xyz)
        slice_idx_xyz = [ix_occ, iy_occ, iz_occ];
    else
        slice_idx_xyz = custom_slice_idx_xyz;
    end

    visualizeMedium3DWithTransducers( ...
        medium.sound_speed, ...
        x_vec, y_vec, z_vec, ...
        elem_pos_bottom, elem_pos_top, ...
        focus_pos, ...
        slice_idx_xyz, ...
        medium3d_slice_alpha, ...
        medium3d_marker_size, ...
        show_focus_in_medium3d);
end

sound_speed_interp = griddedInterpolant( ...
    {x_vec(:), y_vec(:), z_vec(:)}, double(medium.sound_speed), ...
    'linear', 'nearest');

fprintf('\n===== 双阵列共同聚焦：相位计算模式 = %s =====\n', phase_mode);

tau_elem = zeros(num_elem_total, 1);

    tline = linspace(0, 1, num_segments + 1).';
    for n = 1:num_elem_total
        p0 = elem_pos(n, :);
        path = p0 + tline .* (focus_pos - p0);
        c_path = sound_speed_interp(path(:,1), path(:,2), path(:,3));
        c_path(~isfinite(c_path)) = c0;
        c_path(c_path <= 0) = c0;

        dif = diff(path, 1, 1);
        ds  = sqrt(sum(dif.^2, 2));
        c_avg = 0.5 * (c_path(1:end-1) + c_path(2:end));

        tau_elem(n) = sum(ds ./ c_avg);
    end

tau_ref    = min(tau_elem);
delay_elem = tau_elem - tau_ref;
phi_elem = 2*pi*f0 .* delay_elem;
phi_elem = angle(exp(1i * phi_elem));
delay_bottom_us = reshape(delay_elem(bottom_idx), [Nx_elem, Ny_elem]).' * 1e6;
delay_top_us    = reshape(delay_elem(top_idx),    [Nx_elem, Ny_elem]).' * 1e6;
phase_bottom_deg = reshape(phi_elem(bottom_idx), [Nx_elem, Ny_elem]).' * 180/pi;
phase_top_deg    = reshape(phi_elem(top_idx),    [Nx_elem, Ny_elem]).' * 180/pi;

fprintf('全阵列最短传播时间 tau_ref = %.6f us\n', tau_ref*1e6);
fprintf('全阵列最大相对延时 = %.6f us\n', max(delay_elem)*1e6);
fprintf('下阵列最大相对延时 = %.6f us\n', max(delay_elem(bottom_idx))*1e6);
fprintf('上阵列最大相对延时 = %.6f us\n', max(delay_elem(top_idx))*1e6);

figure('Name', 'Delay maps [us]');
tiledlayout(1,2);

nexttile;
imagesc(delay_bottom_us);
axis image; colorbar; colormap(jet);
title('Bottom array delay [\mus]');
xlabel('Element x-index'); ylabel('Element y-index');

nexttile;
imagesc(delay_top_us);
axis image; colorbar; colormap(jet);
title('Top array delay [\mus]');
xlabel('Element x-index'); ylabel('Element y-index');

figure('Name', 'Phase maps [deg]');
tiledlayout(1,2);

nexttile;
imagesc(phase_bottom_deg);
axis image; colorbar; colormap(hsv);
title('Bottom array phase [deg]');
xlabel('Element x-index'); ylabel('Element y-index');

nexttile;
imagesc(phase_top_deg);
axis image; colorbar; colormap(hsv);
title('Top array phase [deg]');
xlabel('Element x-index'); ylabel('Element y-index');

t_arr = kgrid.t_array(:).';
source_signal = zeros(num_elem_total, numel(t_arr), 'single');

for n = 1:num_elem_total
    source_signal(n, :) = single(input_pressure * sin(2*pi*f0*t_arr + phi_elem(n)));
end

source.p_mask = karray.getArrayBinaryMask(kgrid);
source.p      = karray.getDistributedSourceSignal(kgrid, source_signal);
sensor.mask   = [1, 1, 1, Nx, Ny, Nz].';
sensor.record = {'p_max', 'p_rms'};

input_args = { ...
    'PMLSize', 20, ...
    'PMLInside', false, ...
    'DataCast', 'single', ...
    'PlotSim', true, ...
    'DisplayMask', source.p_mask};

switch run_model
    case 1
        sensor_data = kspaceFirstOrder3D(kgrid, medium, source, sensor, input_args{:});
    case 2
        sensor_data = kspaceFirstOrder3DG(kgrid, medium, source, sensor, input_args{:});
    otherwise
        error('run_model 只能取 1 或 2');
end

P3_max = double(sensor_data.p_max);
P3_rms = double(sensor_data.p_rms);

[~, ixf] = min(abs(x_vec - focus_pos(1)));
[~, iyf] = min(abs(y_vec - focus_pos(2)));
[~, izf] = min(abs(z_vec - focus_pos(3)));

x_focus_grid = x_vec(ixf) * 1e3;
y_focus_grid = y_vec(iyf) * 1e3;
z_focus_grid = z_vec(izf) * 1e3;
P_focus_max = P3_max(ixf, iyf, izf);
P_focus_rms = P3_rms(ixf, iyf, izf);

roi_half_mm = 10;
roi_half_m  = roi_half_mm * 1e-3;
x_roi = find(abs(x_vec - focus_pos(1)) <= roi_half_m);
y_roi = find(abs(y_vec - focus_pos(2)) <= roi_half_m);
z_roi = find(abs(z_vec - focus_pos(3)) <= roi_half_m);
P_roi_max = P3_max(x_roi, y_roi, z_roi);
[P_peak_roi, idx_peak_roi] = max(P_roi_max(:));
[ixr, iyr, izr] = ind2sub(size(P_roi_max), idx_peak_roi);

ixp = x_roi(ixr);
iyp = y_roi(iyr);
izp = z_roi(izr);
x_peak = x_vec(ixp) * 1e3;
y_peak = y_vec(iyp) * 1e3;
z_peak = z_vec(izp) * 1e3;
P_peak_rms_roi = P3_rms(ixp, iyp, izp);

[P_peak_global, idx_peak_global] = max(P3_max(:));
[ixg, iyg, izg] = ind2sub(size(P3_max), idx_peak_global);

x_peak_global = x_vec(ixg) * 1e3;
y_peak_global = y_vec(iyg) * 1e3;
z_peak_global = z_vec(izg) * 1e3;
P_peak_global_rms = P3_rms(ixg, iyg, izg);

fprintf('\n===== 仿真结果 =====\n');
fprintf('当前仿真 N = %d\n', num_balls_use);
fprintf('焦点最近网格点 = [%.3f, %.3f, %.3f] mm\n', x_focus_grid, y_focus_grid, z_focus_grid);
fprintf('焦点最近网格点 p_max = %.6e Pa\n', P_focus_max);
fprintf('焦点最近网格点 p_rms = %.6e Pa\n', P_focus_rms);

fprintf('ROI 局部峰值位置 = [%.3f, %.3f, %.3f] mm\n', x_peak, y_peak, z_peak);
fprintf('ROI 局部峰值 p_max = %.6e Pa\n', P_peak_roi);
fprintf('ROI 局部峰值 p_rms = %.6e Pa\n', P_peak_rms_roi);

fprintf('全域峰值位置 = [%.3f, %.3f, %.3f] mm\n', x_peak_global, y_peak_global, z_peak_global);
fprintf('全域峰值 p_max = %.6e Pa\n', P_peak_global);
fprintf('全域峰值 p_rms = %.6e Pa\n', P_peak_global_rms);

figure('Name', 'p_{max} / p_{rms} slices');
tiledlayout(2,3);

nexttile;
imagesc(y_vec*1e3, x_vec*1e3, squeeze(P3_max(:,:,izf)));
axis image; colorbar; colormap(jet);
xlabel('y [mm]'); ylabel('x [mm]');
title(sprintf('p_{max} XY @ z = %.2f mm', z_vec(izf)*1e3));

nexttile;
imagesc(z_vec*1e3, x_vec*1e3, squeeze(P3_max(:,iyf,:)));
axis image; colorbar; colormap(jet);
xlabel('z [mm]'); ylabel('x [mm]');
title(sprintf('p_{max} XZ @ y = %.2f mm', y_vec(iyf)*1e3));

nexttile;
imagesc(z_vec*1e3, y_vec*1e3, squeeze(P3_max(ixf,:,:)));
axis image; colorbar; colormap(jet);
xlabel('z [mm]'); ylabel('y [mm]');
title(sprintf('p_{max} YZ @ x = %.2f mm', x_vec(ixf)*1e3));

nexttile;
imagesc(y_vec*1e3, x_vec*1e3, squeeze(P3_rms(:,:,izf)));
axis image; colorbar; colormap(jet);
xlabel('y [mm]'); ylabel('x [mm]');
title(sprintf('p_{rms} XY @ z = %.2f mm', z_vec(izf)*1e3));

nexttile;
imagesc(z_vec*1e3, x_vec*1e3, squeeze(P3_rms(:,iyf,:)));
axis image; colorbar; colormap(jet);
xlabel('z [mm]'); ylabel('x [mm]');
title(sprintf('p_{rms} XZ @ y = %.2f mm', y_vec(iyf)*1e3));

nexttile;
imagesc(z_vec*1e3, y_vec*1e3, squeeze(P3_rms(ixf,:,:)));
axis image; colorbar; colormap(jet);
xlabel('z [mm]'); ylabel('y [mm]');
title(sprintf('p_{rms} YZ @ x = %.2f mm', x_vec(ixf)*1e3));

save(sprintf('butterfly_ball_center_medium_simulation_N%d.mat', num_balls_use), ...
    'ball_data_mode', 'ball_mat_file', 'ball_csv_file', ...
    'sphere_diameter_m', 'sphere_radius_m', ...
    'num_total_balls', 'num_balls_use', 'selected_ids', ...
    'centers_use_m', 'ball_center_target', 'ball_rot_xyz_deg', ...
    'medium', 'occ_full', ...
    'delay_elem', 'phi_elem', ...
    'sensor_data', ...
    'focus_pos', 'x_vec', 'y_vec', 'z_vec', ...
    'P_focus_max', 'P_focus_rms', ...
    'P_peak_roi', 'x_peak', 'y_peak', 'z_peak');

fprintf('\n已保存: butterfly_ball_center_medium_simulation_N%d.mat\n', num_balls_use);

function [ball_table, sphere_diameter_m] = loadButterflyBallData(ball_data_mode, mat_file, csv_file, sphere_diam_mm_override)
    switch lower(ball_data_mode)
        case 'mat'
            S = load(mat_file);

            if isfield(S, 'ballTable')
                ball_table = S.ballTable;
            elseif isfield(S, 'ballCenters')
                centers = S.ballCenters;
                global_id = (1:size(centers,1)).';
                ball_table = table(global_id, centers(:,1), centers(:,2), centers(:,3), ...
                    'VariableNames', {'global_id','x_mm','y_mm','z_mm'});
            elseif isfield(S, 'ballInfo') && isfield(S.ballInfo, 'allTable')

                ball_table = S.ballInfo.allTable;
            else
                error('MAT 文件中未找到 ballTable / ballCenters / ballInfo.allTable。');
            end

            if ~isempty(sphere_diam_mm_override)
                sphere_diameter_m = sphere_diam_mm_override * 1e-3;
            elseif isfield(S, 'sphereDiameter')
                sphere_diameter_m = double(S.sphereDiameter) * 1e-3;
            elseif isfield(S, 'sphereDiameterInput')
                sphere_diameter_m = double(S.sphereDiameterInput) * 1e-3;
            else
                error('MAT 文件中未找到 sphereDiameter / sphereDiameterInput，请手动指定 sphere_diameter_mm_override。');
            end

        case 'csv'
            ball_table = readtable(csv_file);

            if isempty(sphere_diam_mm_override)
                error('使用 CSV 时必须指定 sphere_diameter_mm_override。');
            end
            sphere_diameter_m = sphere_diam_mm_override * 1e-3;

        otherwise
            error('未知 ball_data_mode。');
    end

    reqVars = {'global_id', 'x_mm', 'y_mm', 'z_mm'};
    for i = 1:numel(reqVars)
        if ~ismember(reqVars{i}, ball_table.Properties.VariableNames)
            error('球数据表缺少必要列: %s', reqVars{i});
        end
    end

    ball_table = sortrows(ball_table, 'global_id');

    if any(~isfinite(ball_table.x_mm)) || any(~isfinite(ball_table.y_mm)) || any(~isfinite(ball_table.z_mm))
        error('球心坐标表中存在 NaN/Inf。');
    end

    if sphere_diameter_m <= 0
        error('球直径必须为正。');
    end
end

function [centers_tfm, bbox_center_raw] = transformBallCenters(centers_raw, center_target, rot_xyz_deg)
    bbox_min = min(centers_raw, [], 1);
    bbox_max = max(centers_raw, [], 1);
    bbox_center_raw = 0.5 * (bbox_min + bbox_max);
    C0 = centers_raw - bbox_center_raw;
    R = buildRotationMatrixXYZ(rot_xyz_deg);
    Crot = (R * C0.').';
    centers_tfm = Crot + center_target;
end

function R = buildRotationMatrixXYZ(rot_xyz_deg)
    ax = deg2rad(rot_xyz_deg(1));
    ay = deg2rad(rot_xyz_deg(2));
    az = deg2rad(rot_xyz_deg(3));
    Rx = [1 0 0;
          0 cos(ax) -sin(ax);
          0 sin(ax)  cos(ax)];

    Ry = [ cos(ay) 0 sin(ay);
           0       1 0;
          -sin(ay) 0 cos(ay)];

    Rz = [cos(az) -sin(az) 0;
          sin(az)  cos(az) 0;
          0        0       1];

    R = Rz * Ry * Rx;
end

function [bbox_ix, bbox_iy, bbox_iz] = getSphereLocalBBoxIndices(center, R, x_vec, y_vec, z_vec, dx, dy, dz, bbox_pad_vox)
    pad_x = R + (bbox_pad_vox + 1) * dx;
    pad_y = R + (bbox_pad_vox + 1) * dy;
    pad_z = R + (bbox_pad_vox + 1) * dz;
    bbox_ix = find(x_vec >= center(1) - pad_x & x_vec <= center(1) + pad_x);
    bbox_iy = find(y_vec >= center(2) - pad_y & y_vec <= center(2) + pad_y);
    bbox_iz = find(z_vec >= center(3) - pad_z & z_vec <= center(3) + pad_z);

    if isempty(bbox_ix) || isempty(bbox_iy) || isempty(bbox_iz)
        error('某个球的局部包围盒映射到网格后为空。');
    end
end

function occ_local = computeSingleSphereOccFraction(center, R, x_loc, y_loc, z_loc, dx, dy, dz, ss_xyz)
    [X, Y, Z] = ndgrid(x_loc, y_loc, z_loc);
    occ_local = zeros(size(X), 'single');
    ox = ((1:ss_xyz(1)) - (ss_xyz(1)+1)/2) * (dx / ss_xyz(1));
    oy = ((1:ss_xyz(2)) - (ss_xyz(2)+1)/2) * (dy / ss_xyz(2));
    oz = ((1:ss_xyz(3)) - (ss_xyz(3)+1)/2) * (dz / ss_xyz(3));
    nsub = prod(ss_xyz);

    for ix = 1:ss_xyz(1)
        for iy = 1:ss_xyz(2)
            for iz = 1:ss_xyz(3)
                dxp = X + ox(ix) - center(1);
                dyp = Y + oy(iy) - center(2);
                dzp = Z + oz(iz) - center(3);
                inside = (dxp.^2 + dyp.^2 + dzp.^2) <= R^2;
                occ_local = occ_local + single(inside);
            end
        end
    end

    occ_local = occ_local / nsub;
end

function [t_array, dt] = makeTimeCFL(dx, dy, dz, c_ref, cfl, t_end)
    dt = cfl * min([dx, dy, dz]) / c_ref;
    Nt = floor(t_end / dt) + 1;
    t_array = (0:Nt-1) * dt;
end

function visualizeMedium3DWithTransducers(sound_speed, x_vec, y_vec, z_vec, ...
    elem_pos_bottom, elem_pos_top, focus_pos, slice_idx_xyz, slice_alpha, marker_size, show_focus)

    Nx = size(sound_speed, 1);
    Ny = size(sound_speed, 2);
    Nz = size(sound_speed, 3);

    ix = clampIndex(slice_idx_xyz(1), 1, Nx);
    iy = clampIndex(slice_idx_xyz(2), 1, Ny);
    iz = clampIndex(slice_idx_xyz(3), 1, Nz);

    [Yg, Xg, Zg] = meshgrid(1:Ny, 1:Nx, 1:Nz);

    figure('Name', 'Medium Sound Speed and Transducer Positions', 'Color', 'w');
    ax = axes('Projection', 'perspective');
    hold(ax, 'on');
    grid(ax, 'on');
    view(ax, 3);

    hs = slice(ax, Yg, Xg, Zg, double(sound_speed), iy, ix, iz);
    set(hs, 'EdgeColor', 'none', 'FaceAlpha', slice_alpha);

    colormap(ax, jet);
    cb = colorbar(ax);
    cb.Label.String = 'Sound speed (m/s)';

    [ix_b, iy_b, iz_b] = physicalPosToGridIndex(elem_pos_bottom, x_vec, y_vec, z_vec);
    [ix_t, iy_t, iz_t] = physicalPosToGridIndex(elem_pos_top,    x_vec, y_vec, z_vec);

    scatter3(ax, iy_b, ix_b, iz_b, marker_size, ...
        'o', 'filled', ...
        'MarkerFaceColor', [0.35 0.05 0.75], ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Bottom transducers');

    scatter3(ax, iy_t, ix_t, iz_t, marker_size, ...
        'o', 'filled', ...
        'MarkerFaceColor', [1.00 0.10 0.10], ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Top transducers');

    if show_focus
        [ix_f, iy_f, iz_f] = physicalPosToGridIndex(focus_pos, x_vec, y_vec, z_vec);
        scatter3(ax, iy_f, ix_f, iz_f, 140, ...
            'p', 'filled', ...
            'MarkerFaceColor', [1.00 0.95 0.10], ...
            'MarkerEdgeColor', 'k', ...
            'DisplayName', 'Focus');
    end

    xlabel(ax, 'Y (grid points)');
    ylabel(ax, 'X (grid points)');
    zlabel(ax, 'Z (grid points)');
    title(ax, 'Medium Sound Speed and Transducer Positions');

    axis(ax, 'tight');
    axis(ax, 'vis3d');
    box(ax, 'on');
    camproj(ax, 'perspective');
    legend(ax, 'Location', 'best');
end

function [ix, iy, iz] = physicalPosToGridIndex(pos_xyz, x_vec, y_vec, z_vec)
    if size(pos_xyz,2) ~= 3
        error('physicalPosToGridIndex: 输入坐标必须是 N×3。');
    end

    n = size(pos_xyz,1);
    ix = zeros(n,1);
    iy = zeros(n,1);
    iz = zeros(n,1);

    for k = 1:n
        [~, ix(k)] = min(abs(x_vec - pos_xyz(k,1)));
        [~, iy(k)] = min(abs(y_vec - pos_xyz(k,2)));
        [~, iz(k)] = min(abs(z_vec - pos_xyz(k,3)));
    end
end

function v = clampIndex(v, vmin, vmax)
    v = round(v);
    v = max(vmin, min(vmax, v));
end
