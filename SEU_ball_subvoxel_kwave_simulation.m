%% SEU 小球亚体素占据率与双阵列 k-Wave 声场仿真
% 普通 SEU CSV -> 球体并集占据率 -> EPS/空气声速、密度场 -> 3D 连续波声场。
% CSV 与本脚本放在同一目录。求解前将 k-Wave 加入 MATLAB 路径。
% run_model = 0 可单独检查几何、介质、阵列与相位图，无需 k-Wave。

clc; clearvars;

%% 1. 仿真与几何参数
run_model = 2;                    % 0 = 预览，1 = MATLAB CPU，2 = k-Wave CUDA
phase_mode = 'path_integral';     % 'air' 或 'path_integral'
cfl = 0.3;
t_end = 0.6e-3;                   % [s] 沿用小船脚本
record_start_s = 0;               % [s] 若只统计后段，设为已确认的稳态起始时间
plot_solver_progress = true;      % MATLAB CPU 求解过程图

script_dir = fileparts(mfilename('fullpath'));
ball_csv_file = fullfile(script_dir, 'seu_ball_numbering.csv');
sphere_diameter_mm = 2.00;
expected_pitch_mm = 2.10;
num_balls_to_apply = inf;         % inf = 全部 66 球，0 = 纯空气，其他值按 global_id 取前 k 球
ball_center_target_mm = [0, 0, 0]; % 球阵包围盒中心在仿真坐标中的位置

% 空气与 EPS 等效声学参数，沿用小船脚本。
c0 = 346;                         % [m/s]
rho0 = 1.2;                       % [kg/m^3]
c_obj = 500;                      % [m/s]
rho_obj = 25;                     % [kg/m^3]

% 计算域与亚体素设置
dx = 0.5e-3; dy = 0.5e-3; dz = 0.5e-3;
Nx = 401; Ny = 401; Nz = 341;
pml_size = 20;                    % 外置 PML 的单侧网格点数
ss_xyz = [5, 5, 5];

% 双 16×16 对置阵列，使用小船脚本中的声源和预设焦点。
f0 = 40e3;                        % [Hz]
input_pressure = 1;               % [Pa] 单阵元激励幅值
Nx_elem = 16; Ny_elem = 16;
elem_diam = 9e-3;                 % [m]
pitch = 10e-3;                    % [m] 阵元中心距
z_bottom = -84e-3; z_top = 84e-3;  % [m]
x0 = 0; y0 = 0;                   % [m] 阵列轴线在 XY 平面的位置
focus_pos = [0, 5, 2] * 1e-3;     % [m] 仿真坐标
roi_half_mm = 10;                 % 焦点局部峰值搜索范围

%% 2. 可视化选项
show_ball_before_after = true;
show_ball_centers_plot = true;
show_occupancy_slices = true;
show_medium_slices = true;
show_mask_isosurface = true;
show_medium_3d_overview = true;
medium3d_slice_alpha = 0.22;
medium3d_marker_size = 85;
show_focus_in_medium3d = true;
use_occ_centroid_slice = true;
custom_slice_idx_xyz = [];        % [ix iy iz]，留空时用占据率质心

%% 3. 输入检查、网格与坐标变换
validateattributes(run_model, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 2});
phase_mode = validatestring(phase_mode, {'air', 'path_integral'});
validateattributes(ss_xyz, {'numeric'}, {'size', [1, 3], 'integer', 'positive'});
validateattributes([Nx, Ny, Nz], {'numeric'}, {'integer', '>=', 3});
validateattributes([dx, dy, dz, c0, rho0, c_obj, rho_obj, f0, ...
    cfl, t_end, sphere_diameter_mm], {'numeric'}, {'finite', 'positive'});
validateattributes(record_start_s, {'numeric'}, {'scalar', '>=', 0, '<', t_end});
if run_model > 0
    solver_name = 'kspaceFirstOrder3D';
    if run_model == 2
        solver_name = 'kspaceFirstOrder3DG';
    end
    if isempty(which('kWaveGrid')) || isempty(which('kWaveArray')) || isempty(which(solver_name))
        error('请先将 k-Wave 加入 MATLAB 路径。仅预览可将 run_model 设为 0。');
    end
    kgrid = kWaveGrid(Nx, dx, Ny, dy, Nz, dz);
    grid_vec = {kgrid.x_vec(:), kgrid.y_vec(:), kgrid.z_vec(:)};
else
    % k-Wave 的坐标原点位于每轴索引 floor(N/2)+1。
    grid_vec = {((0:Nx-1).' - floor(Nx/2))*dx, ...
                ((0:Ny-1).' - floor(Ny/2))*dy, ...
                ((0:Nz-1).' - floor(Nz/2))*dz};
end
grid_size = [Nx, Ny, Nz];
grid_step = [dx, dy, dz];
grid_min = cellfun(@(v) v(1), grid_vec);
grid_max = cellfun(@(v) v(end), grid_vec);
grid_mm = cellfun(@(v) v*1e3, grid_vec, 'UniformOutput', false);
x_vec = grid_vec{1}; y_vec = grid_vec{2}; z_vec = grid_vec{3};

ball_table_all = loadSEUBallTable(ball_csv_file, expected_pitch_mm);
centers_raw_mm = [ball_table_all.x_mm, ball_table_all.y_mm, ball_table_all.z_mm];
num_total_balls = height(ball_table_all);
if isequal(num_balls_to_apply, inf)
    num_balls_use = num_total_balls;
else
    validateattributes(num_balls_to_apply, {'numeric'}, ...
        {'scalar', 'integer', '>=', 0, '<=', num_total_balls});
    num_balls_use = num_balls_to_apply;
end

% 原 CSV 的 Y 是高度，原 Z 是字母竖向。绕 X 轴 +90° 后，仿真 Z 为高度。
% 对全部 66 球只定位一次，增减球数时球阵的坐标参考系保持一致。
raw_bbox_center = (min(centers_raw_mm, [], 1) + max(centers_raw_mm, [], 1))/2;
centers_local_mm = centers_raw_mm - raw_bbox_center;
centers_sim_mm = [centers_local_mm(:,1), -centers_local_mm(:,3), centers_local_mm(:,2)] ...
    + ball_center_target_mm;
selected_rows = 1:num_balls_use;
selected_ids = ball_table_all.global_id(selected_rows);
selected_letters = ball_table_all.source_curves(selected_rows);
centers_use_m = centers_sim_mm(selected_rows, :) * 1e-3;
sphere_radius_m = sphere_diameter_mm * 0.5e-3;

if sphere_diameter_mm > expected_pitch_mm + 1e-8
    error('球径 %.3f mm 大于最小球心距 %.3f mm。', sphere_diameter_mm, expected_pitch_mm);
end
ball_min = (min(centers_sim_mm, [], 1) - sphere_diameter_mm/2)*1e-3;
ball_max = (max(centers_sim_mm, [], 1) + sphere_diameter_mm/2)*1e-3;
if any(ball_min < grid_min) || any(ball_max > grid_max)
    error('SEU 球阵超出仿真网格，请检查位置和计算域。');
end

fprintf('球心文件：%s\n', ball_csv_file);
fprintf('SEU 球数 %d/%d，球径 %.2f mm，点阵间距 %.2f mm\n', ...
    num_balls_use, num_total_balls, sphere_diameter_mm, expected_pitch_mm);
fprintf('网格 %d × %d × %d，单个 single 场 %.1f MiB\n', ...
    grid_size, prod(grid_size)*4/1024^2);
fprintf('空气波长 %.3f mm，阵元间距/波长 %.3f\n', c0/f0*1e3, pitch*f0/c0);

%% 4. 局部亚体素并集、占据率与介质场
% 只在 SEU 局部包围盒中分配亚体素网格。包围盒额外保留一层空气节点，
% 使后续局部线性插值在包围盒边缘与全域空气声速相接。
[occ_local, local_idx] = buildSphereOccupancy(centers_use_m, sphere_radius_m, ...
    grid_vec, grid_step, ss_xyz);

occ_full = zeros(grid_size, 'single');
medium.sound_speed = repmat(single(c0), grid_size);
medium.density = repmat(single(rho0), grid_size);
occ_centroid = ball_center_target_mm * 1e-3;
if num_balls_use > 0
    occ_full(local_idx{:}) = occ_local;
    medium.sound_speed(local_idx{:}) = single(c0) + occ_local .* single(c_obj - c0);
    medium.density(local_idx{:}) = single(rho0) + occ_local .* single(rho_obj - rho0);

    occ_weight = sum(occ_local(:), 'double');
    if occ_weight == 0
        error('亚体素采样后占据率全为零，请检查球径和网格分辨率。');
    end
    occ_volume_mm3 = occ_weight * prod(grid_step) * 1e9;
    expected_volume_mm3 = num_balls_use * (4*pi/3) * (sphere_diameter_mm/2)^3;
    fprintf('亚体素采样 %d × %d × %d，非零占据体素 %d\n', ss_xyz, nnz(occ_local));
    fprintf('球体积 %.4f mm^3，理论值 %.4f mm^3，相对偏差 %+.3f%%\n', ...
        occ_volume_mm3, expected_volume_mm3, 100*(occ_volume_mm3/expected_volume_mm3 - 1));

    local_grid = cellfun(@(v, i) v(i), grid_vec, local_idx, 'UniformOutput', false);
    weights = {sum(sum(occ_local, 2, 'double'), 3), ...
               sum(sum(occ_local, 1, 'double'), 3), ...
               sum(sum(occ_local, 1, 'double'), 2)};
    for axis_index = 1:3
        occ_centroid(axis_index) = dot(local_grid{axis_index}, weights{axis_index}(:)) / occ_weight;
    end
    fprintf('占据率质心 [%.3f, %.3f, %.3f] mm\n', occ_centroid*1e3);
end

occ_slice_idx = nearestGridIndices(grid_vec, occ_centroid);
letter_slice_idx = nearestGridIndices(grid_vec, ball_center_target_mm*1e-3);
% XY 截面固定在字母平面，另两面穿过占据率质心。
occ_slice_idx(3) = letter_slice_idx(3);
detail_margin_m = 4e-3;
detail_limits_mm = [max(grid_min, ball_min-detail_margin_m); ...
                    min(grid_max, ball_max+detail_margin_m)].' * 1e3;

%% 5. 球心、占据率、声速和密度可视化
if show_ball_before_after
    figure('Name', 'SEU ball centers before and after transform', 'Color', 'w');
    tiledlayout(1, 2);
    ax = nexttile;
    drawBallCenters(ax, centers_raw_mm, ball_table_all.source_curves);
    title(ax, 'CSV coordinates (Y is height)');
    ax = nexttile;
    drawBallCenters(ax, centers_sim_mm, ball_table_all.source_curves);
    plot3(ax, ball_center_target_mm(1), ball_center_target_mm(2), ...
        ball_center_target_mm(3), 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 12);
    title(ax, 'Simulation coordinates (Z is height)');
end

if show_ball_centers_plot
    figure('Name', 'Selected SEU ball centers', 'Color', 'w');
    ax = axes;
    drawBallCenters(ax, centers_use_m*1e3, selected_letters);
    view(ax, 2);
    set(ax, 'YDir', 'reverse');
    title(ax, sprintf('SEU in the XY plane (%d balls)', num_balls_use));
    if num_balls_use > 0
        legend(ax, 'Location', 'bestoutside');
    end
end

if show_occupancy_slices
    figure('Name', 'SEU occupancy fraction slices', 'Color', 'w');
    tiledlayout(1, 3);
    plotFieldSlices(occ_full, grid_mm, occ_slice_idx, 'EPS fraction', parula(256), ...
        [0, 1], detail_limits_mm);
end

if show_medium_slices
    figure('Name', 'SEU sound speed and density slices', 'Color', 'w');
    tiledlayout(2, 3);
    plotFieldSlices(medium.sound_speed, grid_mm, occ_slice_idx, 'c [m/s]', jet(256), ...
        sort([c0, c_obj]), detail_limits_mm);
    plotFieldSlices(medium.density, grid_mm, occ_slice_idx, '\rho [kg/m^3]', jet(256), ...
        sort([rho0, rho_obj]), detail_limits_mm);
end

if show_mask_isosurface && num_balls_use > 0 && max(occ_local(:)) > 0.5
    figure('Name', 'SEU occupancy isosurface (0.5)', 'Color', 'w');
    ax = axes;
    % isosurface 的前两维按 [Y, X] 排列。
    occ_vis = permute(occ_local, [2, 1, 3]);
    coords = cellfun(@(v) v*1e3, local_grid, 'UniformOutput', false);
    p = patch(ax, isosurface(coords{:}, occ_vis, 0.5), ...
        'FaceColor', [0.2, 0.8, 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    isonormals(coords{:}, occ_vis, p);
    hold(ax, 'on');
    plot3(ax, centers_use_m(:,1)*1e3, centers_use_m(:,2)*1e3, centers_use_m(:,3)*1e3, 'k.');
    plot3(ax, occ_centroid(1)*1e3, occ_centroid(2)*1e3, occ_centroid(3)*1e3, ...
        'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 12);
    formatSpatialAxes(ax);
    camlight(ax); lighting(ax, 'gouraud');
    title(ax, 'SEU EPS fraction = 0.5');
end

%% 6. 对置阵列、焦点与三维非均匀介质总览
[elem_x, elem_y] = ndgrid(((0:Nx_elem-1) - (Nx_elem-1)/2)*pitch + x0, ...
                        ((0:Ny_elem-1) - (Ny_elem-1)/2)*pitch + y0);
num_elem_single = numel(elem_x);
elem_pos_bottom = [elem_x(:), elem_y(:), repmat(z_bottom, num_elem_single, 1)];
elem_pos_top = [elem_x(:), elem_y(:), repmat(z_top, num_elem_single, 1)];
elem_pos = [elem_pos_bottom; elem_pos_top];
num_elem_total = size(elem_pos, 1);
elem_extent = [elem_diam/2, elem_diam/2, 0];
if any(min(elem_pos, [], 1) - elem_extent < grid_min) || ...
        any(max(elem_pos, [], 1) + elem_extent > grid_max)
    error('双阵列超出计算域，请检查阵元坐标和网格范围。');
end
if any(focus_pos < grid_min) || any(focus_pos > grid_max)
    error('焦点超出计算域。');
end
focus_idx = nearestGridIndices(grid_vec, focus_pos);

fprintf('双阵列共 %d 个阵元，阵列间距 %.1f mm，焦点 [%.1f, %.1f, %.1f] mm\n', ...
    num_elem_total, (z_top-z_bottom)*1e3, focus_pos*1e3);

figure('Name', 'Opposed arrays, focus and SEU', 'Color', 'w');
ax = axes;
plotArrayAndFocus(ax, elem_pos_bottom, elem_pos_top, focus_pos, 32);
if num_balls_use > 0
    scatter3(ax, centers_use_m(:,1)*1e3, centers_use_m(:,2)*1e3, ...
        centers_use_m(:,3)*1e3, 16, 'filled', 'DisplayName', 'SEU ball centers');
end
formatSpatialAxes(ax);
legend(ax, 'Location', 'best');

if show_medium_3d_overview
    slice_idx_xyz = nearestGridIndices(grid_vec, occ_centroid);
    if ~use_occ_centroid_slice && ~isempty(custom_slice_idx_xyz)
        validateattributes(custom_slice_idx_xyz, {'numeric'}, {'size', [1, 3], 'integer', 'positive'});
        if any(custom_slice_idx_xyz > grid_size)
            error('custom_slice_idx_xyz 超出网格。');
        end
        slice_idx_xyz = custom_slice_idx_xyz;
    end
    visualizeMedium3DWithTransducers(medium.sound_speed, grid_mm, ...
        elem_pos_bottom, elem_pos_top, focus_pos, slice_idx_xyz, ...
        medium3d_slice_alpha, medium3d_marker_size, show_focus_in_medium3d, c0, c_obj);
end

%% 7. 空气传播时延或路径积分相位
ray_vectors = focus_pos - elem_pos;
ray_lengths = vecnorm(ray_vectors, 2, 2);
tau_elem = ray_lengths / c0;
if strcmp(phase_mode, 'path_integral') && num_balls_use > 0
    c_interp = griddedInterpolant(local_grid, medium.sound_speed(local_idx{:}), 'linear', 'none');
    max_ray_step = min(grid_step)/4;
    for n = 1:num_elem_total
        if ray_lengths(n) == 0
            continue;
        end
        nseg = ceil(ray_lengths(n)/max_ray_step);
        fraction_along_ray = linspace(0, 1, nseg+1).';
        path = elem_pos(n,:) + fraction_along_ray .* ray_vectors(n,:);
        c_path = double(c_interp(path(:,1), path(:,2), path(:,3)));
        c_path(isnan(c_path)) = c0;   % 局部包围盒外为均匀空气
        tau_elem(n) = ray_lengths(n) * trapz(fraction_along_ray, 1 ./ c_path);
    end
end

delay_elem = tau_elem - min(tau_elem);
phi_elem = angle(exp(1i * 2*pi*f0 * delay_elem));
fprintf('相位模式 %s，最大相对延时 %.3f us\n', phase_mode, max(delay_elem)*1e6);
plotElementMaps(delay_elem*1e6, Nx_elem, Ny_elem, 'Array delay [us]', jet(256));
plotElementMaps(rad2deg(phi_elem), Nx_elem, Ny_elem, 'Array phase [deg]', hsv(256));

if run_model == 0
    fprintf('几何、介质、阵列与相位预览完成。\n');
    return;
end

%% 8. k-Wave 声源、传感器与求解
% 两种介质工况统一使用同一 dt，以便对比纯空气和含 SEU 的结果。
dt = cfl * min(grid_step) / max(c0, c_obj);
Nt = floor(t_end/dt) + 1;
kgrid.setTime(Nt, dt);
t_arr = kgrid.t_array(:).';
fprintf('k-Wave 时间步 %.3e s，总步数 %d\n', dt, Nt);

karray = kWaveArray;
for n = 1:num_elem_total
    face_pos = elem_pos(n,:);
    if n <= num_elem_single
        face_pos(3) = face_pos(3) + 1e-3;
    else
        face_pos(3) = face_pos(3) - 1e-3;
    end
    karray.addDiscElement(elem_pos(n,:), elem_diam, face_pos);
end
source_signal = single(input_pressure * sin(2*pi*f0*t_arr + phi_elem));
source.p_mask = karray.getArrayBinaryMask(kgrid);
source.p = karray.getDistributedSourceSignal(kgrid, source_signal);

sensor.mask = [1; 1; 1; Nx; Ny; Nz];
sensor.record = {'p_max', 'p_rms'};
sensor.record_start_index = find(t_arr >= record_start_s, 1);
if isempty(sensor.record_start_index)
    error('record_start_s 晚于 k-Wave 最后一个时间步。');
end
input_args = {'PMLSize', pml_size, 'PMLInside', false, 'DataCast', 'single', ...
    'Smooth', false, 'PlotSim', plot_solver_progress, 'DisplayMask', source.p_mask};
if run_model == 1
    sensor_data = kspaceFirstOrder3D(kgrid, medium, source, sensor, input_args{:});
else
    sensor_data = kspaceFirstOrder3DG(kgrid, medium, source, sensor, input_args{:});
end

%% 9. 焦点、局部峰值、全域峰值与声压切片
P3_max = sensor_data.p_max;
P3_rms = sensor_data.p_rms;
ixf = focus_idx(1); iyf = focus_idx(2); izf = focus_idx(3);
P_focus_max = P3_max(ixf, iyf, izf);
P_focus_rms = P3_rms(ixf, iyf, izf);
focus_grid_mm = [x_vec(ixf), y_vec(iyf), z_vec(izf)]*1e3;

roi_half_m = roi_half_mm*1e-3;
x_roi = find(abs(x_vec - focus_pos(1)) <= roi_half_m);
y_roi = find(abs(y_vec - focus_pos(2)) <= roi_half_m);
z_roi = find(abs(z_vec - focus_pos(3)) <= roi_half_m);
P_roi_max = P3_max(x_roi, y_roi, z_roi);
[P_peak_roi, idx_peak_roi] = max(P_roi_max(:));
[ixr, iyr, izr] = ind2sub(size(P_roi_max), idx_peak_roi);
roi_peak_mm = [x_vec(x_roi(ixr)), y_vec(y_roi(iyr)), z_vec(z_roi(izr))]*1e3;
P_peak_rms_roi = P3_rms(x_roi(ixr), y_roi(iyr), z_roi(izr));

[P_peak_global, idx_peak_global] = max(P3_max(:));
[ixg, iyg, izg] = ind2sub(grid_size, idx_peak_global);
global_peak_mm = [x_vec(ixg), y_vec(iyg), z_vec(izg)]*1e3;
P_peak_global_rms = P3_rms(ixg, iyg, izg);

fprintf('\n焦点网格位置 [%.3f, %.3f, %.3f] mm，p_max %.6g Pa，p_rms %.6g Pa\n', ...
    focus_grid_mm, P_focus_max, P_focus_rms);
fprintf('焦点 ROI 峰值 [%.3f, %.3f, %.3f] mm，p_max %.6g Pa，p_rms %.6g Pa\n', ...
    roi_peak_mm, P_peak_roi, P_peak_rms_roi);
fprintf('ROI 峰值与目标焦点距离 %.3f mm\n', norm(roi_peak_mm - focus_pos*1e3));
fprintf('全域峰值 [%.3f, %.3f, %.3f] mm，p_max %.6g Pa，p_rms %.6g Pa\n', ...
    global_peak_mm, P_peak_global, P_peak_global_rms);

figure('Name', 'Pressure slices through the focus', 'Color', 'w');
tiledlayout(2, 3);
plotFieldSlices(P3_max, grid_mm, focus_idx, 'p_{max} [Pa]', jet(256), [], []);
plotFieldSlices(P3_rms, grid_mm, focus_idx, 'p_{rms} [Pa]', jet(256), [], []);

figure('Name', 'Pressure in the SEU plane', 'Color', 'w');
tiledlayout(1, 2);
iz_letter = letter_slice_idx(3);
plotFieldSlice(nexttile, grid_mm{1}, grid_mm{2}, P3_max(:,:,iz_letter).', ...
    'X', 'Y', sprintf('p_{max} at Z = %.2f mm', grid_mm{3}(iz_letter)), ...
    'Pa', jet(256), [], detail_limits_mm([1, 2],:));
plotFieldSlice(nexttile, grid_mm{1}, grid_mm{2}, P3_rms(:,:,iz_letter).', ...
    'X', 'Y', sprintf('p_{rms} at Z = %.2f mm', grid_mm{3}(iz_letter)), ...
    'Pa', jet(256), [], detail_limits_mm([1, 2],:));

%% 局部函数
function T = loadSEUBallTable(csv_file, expected_pitch_mm)
    if ~isfile(csv_file)
        error('找不到 SEU 球心文件：%s', csv_file);
    end
    T = readtable(csv_file, 'TextType', 'string');
    required = {'global_id', 'source_curves', 'x_mm', 'y_mm', 'z_mm'};
    if ~all(ismember(required, T.Properties.VariableNames))
        error('SEU CSV 缺少列：%s', strjoin(setdiff(required, T.Properties.VariableNames), ', '));
    end
    T = sortrows(T, 'global_id');
    xyz = [T.x_mm, T.y_mm, T.z_mm];
    if ~isequal(T.global_id, (1:66).') || ...
            ~isequal([sum(T.source_curves == "S"), sum(T.source_curves == "E"), ...
                      sum(T.source_curves == "U")], [23, 23, 20])
        error('普通 SEU CSV 应包含 global_id 1–66，其中 S/E/U 分别有 23/23/20 球。');
    end
    if ~isnumeric(xyz) || any(~isfinite(xyz(:))) || any(abs(T.y_mm) > 1e-8)
        error('普通 SEU CSV 的球心必须为有限数值，并位于 Y=0 平面。');
    end
    min_distance = inf;
    for i = 1:height(T)-1
        distances = vecnorm(xyz(i+1:end,:) - xyz(i,:), 2, 2);
        min_distance = min(min_distance, min(distances));
    end
    if abs(min_distance - expected_pitch_mm) > 1e-8
        error('最小球心距为 %.4f mm，预期 %.2f mm，请确认使用普通 SEU CSV。', ...
            min_distance, expected_pitch_mm);
    end
end

function [occ_local, local_idx] = buildSphereOccupancy(centers, radius, grid_vec, step, ss)
    occ_local = [];
    local_idx = {};
    if isempty(centers)
        return;
    end
    bounds_min = min(centers, [], 1) - radius;
    bounds_max = max(centers, [], 1) + radius;
    local_idx = cell(1, 3);
    subgrid = cell(1, 3);
    n_coarse = zeros(1, 3);
    for d = 1:3
        v = grid_vec{d};
        local_idx{d} = find(v >= bounds_min(d) - 1.5*step(d) & ...
                            v <= bounds_max(d) + 1.5*step(d));
        n_coarse(d) = numel(local_idx{d});
        offsets = ((1:ss(d)).' - (ss(d)+1)/2) * step(d)/ss(d);
        subgrid{d} = reshape(v(local_idx{d}).' + offsets, [], 1);
    end

    occupied_subpoints = false(n_coarse .* ss);
    for k = 1:size(centers, 1)
        delta = cellfun(@(v, c) v-c, subgrid, num2cell(centers(k,:)), 'UniformOutput', false);
        ix = find(abs(delta{1}) <= radius);
        iy = find(abs(delta{2}) <= radius);
        iz = find(abs(delta{3}) <= radius);
        inside = delta{1}(ix).^2 + delta{2}(iy).'.^2 + reshape(delta{3}(iz).^2, 1, 1, []) ...
            <= radius^2;
        occupied_subpoints(ix, iy, iz) = occupied_subpoints(ix, iy, iz) | inside;
    end

    % 每一轴先排子采样索引，再排该轴的粗体素索引。
    counts = reshape(occupied_subpoints, ss(1), n_coarse(1), ...
        ss(2), n_coarse(2), ss(3), n_coarse(3));
    counts = sum(sum(sum(counts, 1), 3), 5);
    occ_local = single(reshape(counts, n_coarse)) / prod(ss);
end

function idx = nearestGridIndices(grid_vec, pos)
    idx = zeros(1, 3);
    for d = 1:3
        [~, idx(d)] = min(abs(grid_vec{d} - pos(d)));
    end
end

function drawBallCenters(ax, xyz_mm, letters)
    hold(ax, 'on');
    letter_names = ["S", "E", "U"];
    colors = lines(3);
    for j = 1:3
        mask = letters == letter_names(j);
        if any(mask)
            scatter3(ax, xyz_mm(mask,1), xyz_mm(mask,2), xyz_mm(mask,3), ...
                24, colors(j,:), 'filled', 'DisplayName', letter_names(j));
        end
    end
    formatSpatialAxes(ax);
end

function formatSpatialAxes(ax)
    axis(ax, 'equal'); grid(ax, 'on'); view(ax, 3);
    xlabel(ax, 'X [mm]'); ylabel(ax, 'Y [mm]'); zlabel(ax, 'Z [mm]');
end

function plotFieldSlices(field, grid_mm, idx, label, cmap, color_limits, limits)
    ix = idx(1); iy = idx(2); iz = idx(3);
    x = grid_mm{1}; y = grid_mm{2}; z = grid_mm{3};
    plotFieldSlice(nexttile, x, y, field(:,:,iz).', 'X', 'Y', ...
        sprintf('XY at Z = %.2f mm', z(iz)), label, cmap, color_limits, sliceLimits(limits, [1, 2]));
    plotFieldSlice(nexttile, x, z, squeeze(field(:,iy,:)).', 'X', 'Z', ...
        sprintf('XZ at Y = %.2f mm', y(iy)), label, cmap, color_limits, sliceLimits(limits, [1, 3]));
    plotFieldSlice(nexttile, y, z, squeeze(field(ix,:,:)).', 'Y', 'Z', ...
        sprintf('YZ at X = %.2f mm', x(ix)), label, cmap, color_limits, sliceLimits(limits, [2, 3]));
end

function out = sliceLimits(limits, dims)
    out = [];
    if ~isempty(limits)
        out = limits(dims,:);
    end
end

function plotFieldSlice(ax, h, v, data, h_axis, v_axis, ttl, label, cmap, color_limits, limits)
    imagesc(ax, h, v, data);
    axis(ax, 'image');
    if strcmp(v_axis, 'Z')
        set(ax, 'YDir', 'normal');
    else
        % 仿真 -Y 对应原 CSV +Z，保持字母正立。
        set(ax, 'YDir', 'reverse');
    end
    if ~isempty(limits)
        xlim(ax, limits(1,:)); ylim(ax, limits(2,:));
    end
    colormap(ax, cmap);
    if ~isempty(color_limits) && color_limits(1) < color_limits(2)
        clim(ax, color_limits);
    end
    cb = colorbar(ax);
    cb.Label.String = label;
    xlabel(ax, [h_axis, ' [mm]']); ylabel(ax, [v_axis, ' [mm]']);
    title(ax, ttl);
end

function plotArrayAndFocus(ax, bottom, top, focus, marker_size)
    hold(ax, 'on');
    scatter3(ax, bottom(:,1)*1e3, bottom(:,2)*1e3, bottom(:,3)*1e3, ...
        marker_size, [0.35, 0.05, 0.75], 'filled', 'DisplayName', 'Bottom transducers');
    scatter3(ax, top(:,1)*1e3, top(:,2)*1e3, top(:,3)*1e3, ...
        marker_size, [1.00, 0.10, 0.10], 'filled', 'DisplayName', 'Top transducers');
    if ~isempty(focus)
        plot3(ax, focus(1)*1e3, focus(2)*1e3, focus(3)*1e3, ...
            'kp', 'MarkerSize', 13, 'MarkerFaceColor', 'y', 'DisplayName', 'Focus');
    end
end

function visualizeMedium3DWithTransducers(sound_speed, grid_mm, bottom, top, focus, ...
    idx, alpha, marker_size, show_focus, c0, c_obj)
    figure('Name', '3D heterogeneous medium and transducers', 'Color', 'w');
    ax = axes('Projection', 'perspective');
    hold(ax, 'on');
    x = grid_mm{1}; y = grid_mm{2}; z = grid_mm{3};
    ix = idx(1); iy = idx(2); iz = idx(3);
    % 只构造三个二维切片的坐标，避免为显示分配全域三维坐标数组。
    [X, Y] = meshgrid(x, y);
    surf(ax, X, Y, z(iz)*ones(size(X)), sound_speed(:,:,iz).', ...
        'EdgeColor', 'none', 'FaceAlpha', alpha, 'HandleVisibility', 'off');
    [X, Z] = meshgrid(x, z);
    surf(ax, X, y(iy)*ones(size(X)), Z, squeeze(sound_speed(:,iy,:)).', ...
        'EdgeColor', 'none', 'FaceAlpha', alpha, 'HandleVisibility', 'off');
    [Y, Z] = meshgrid(y, z);
    surf(ax, x(ix)*ones(size(Y)), Y, Z, squeeze(sound_speed(ix,:,:)).', ...
        'EdgeColor', 'none', 'FaceAlpha', alpha, 'HandleVisibility', 'off');
    if ~show_focus
        focus = [];
    end
    plotArrayAndFocus(ax, bottom, top, focus, marker_size);
    formatSpatialAxes(ax);
    axis(ax, 'vis3d'); box(ax, 'on');
    colormap(ax, jet(256));
    if c0 ~= c_obj
        clim(ax, sort([c0, c_obj]));
    end
    cb = colorbar(ax);
    cb.Label.String = 'Sound speed [m/s]';
    title(ax, 'SEU heterogeneous medium');
    legend(ax, 'Location', 'best');
end

function plotElementMaps(values, nx, ny, label, cmap)
    figure('Name', label, 'Color', 'w');
    tiledlayout(1, 2);
    range = [min(values), max(values)];
    names = {'Bottom', 'Top'};
    for k = 1:2
        ax = nexttile;
        idx = (1:nx*ny) + (k-1)*nx*ny;
        imagesc(ax, reshape(values(idx), nx, ny).');
        axis(ax, 'image');
        colormap(ax, cmap);
        if range(1) < range(2)
            clim(ax, range);
        end
        colorbar(ax);
        xlabel(ax, 'Element X index'); ylabel(ax, 'Element Y index');
        title(ax, [names{k}, ' ', label]);
    end
end
