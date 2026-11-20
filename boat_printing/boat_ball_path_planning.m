clc; clear;
script_dir = fileparts(mfilename('fullpath'));
% close all;

%  coordination_move_from_csv.m

csvFile = fullfile(script_dir, 'boat_ball_numbering.csv');

if ~isfile(csvFile)
    [fn, fp] = uigetfile({'*.csv', 'CSV files (*.csv)'}, '请选择小球中心坐标 CSV 文件');
    if isequal(fn, 0)
        error('未选择 CSV 文件，程序终止。');
    end
    csvFile = fullfile(fp, fn);
end

csvCoordUnit = 'mm';
swapYZColumns = true;
modelOffset_m = [0.0000, 0.0840, 0.0000];
startPoint = [-0.030, 0.0840, 0.0000];
placementOrderMode = 'auto_closure_risk';

sphereR = 0.95e-3;
allowedSteps = [0.0005, 0.0002, 0.0001];   % 0.5 / 0.2 / 0.1 mm
gridBase     = min(allowedSteps);
safetyMargin = 0e-3;
clearance    = 2 * sphereR + safetyMargin;

motionMode = 'multi_axis';
enableFinalApproach = true;
approachParentMode = 'nearest_placed_contact';
contactTolerance = 0.2e-3;
finalApproachDistance = 3.0e-3;
finalApproachStep     = 0.1e-3;
multiAxisPathStep = 0.2e-3;
finalApproachFailMode = 'fallback_normal';
enableMultiAxisAstar = true;

pathOutputDecimals = 6;

plannerOptions.motionMode = motionMode;
plannerOptions.enableFinalApproach = enableFinalApproach;
plannerOptions.approachParentMode = approachParentMode;
plannerOptions.contactTolerance = contactTolerance;
plannerOptions.finalApproachDistance = finalApproachDistance;
plannerOptions.finalApproachStep = finalApproachStep;
plannerOptions.multiAxisPathStep = multiAxisPathStep;
plannerOptions.finalApproachFailMode = finalApproachFailMode;
plannerOptions.enableMultiAxisAstar = enableMultiAxisAstar;

enableRodObstacle = false;
rod.center = [0.0000, 0.0825, 0.0000];
rod.size   = [1.5e-3, 1.5e-3,50e-3];
rod.heightAxis = 2;
rod.longAxis   = 3;
rod.collisionMargin = safetyMargin;
rod.extraHeightGap  = 2.0e-3;
rod.aboveBufferX = sphereR + safetyMargin;
rod.aboveBufferZ = sphereR + safetyMargin;
rod.verticalTolerance = 1e-9;
rod.targetAboveRodMode = 'allow_lower_target';
rod = finalizeRodOptions(rod, sphereR);
plannerOptions.enableRodObstacle = enableRodObstacle;
plannerOptions.rod = rod;

enableStartHorizontalConstraint = true;
startConstraint.startPoint = startPoint;
startConstraint.heightAxis = 2;
startConstraint.forwardAxis = 1;
startConstraint.forwardDirection = 1;
startConstraint.forwardDistance = 4.0e-3;
startConstraint.verticalTolerance = 1e-9;
startConstraint.pathStep = multiAxisPathStep;
startConstraint = finalizeStartHorizontalConstraint(startConstraint);
plannerOptions.enableStartHorizontalConstraint = enableStartHorizontalConstraint;
plannerOptions.startConstraint = startConstraint;

enableSupportObjects = true;
supportConfig.targetGlobalIDs = [12, 36];
supportConfig.saddleTopGap = 0.08e-3;
supportConfig.saddleSize = [1.2e-3, 0.30e-3, 1.2e-3];
supportConfig.enableRails = false;
supportConfig.railWidthX  = 0.25e-3;
supportConfig.railHeightY = 0.45e-3;
supportConfig.railOffsetX = 0.85e-3;
supportConfig.railLengthZ = supportConfig.saddleSize(3);
supportConfig.armWidthZ = 0.90e-3;
supportConfig.armThicknessY = 0.75e-3;
supportConfig.armClearGapToSaddle = 0.08e-3;
supportConfig.riserSizeXZ = [1.1e-3, 1.1e-3];
supportConfig.supportMargin = 0e-3;
supportConfig.contactAllowance = 0.10e-3;
supportConfig.endpointRelaxRadius = 0.30e-3;
supportConfig.visualAlpha = 0.55;
plannerOptions.enableSupportObjects = false;
plannerOptions.supportObjects = [];
plannerOptions.supportConfig = supportConfig;

outFolder = 'CSV_Sphere_JSON_MultiAxis_ObstacleAvoid';
if ~exist(outFolder, 'dir')
    mkdir(outFolder);
end

doPlot = true;
showPointLabels = true;
plotAllPaths = true;
maxPathsToPlot = inf;

csvScale = unitToMeterScale(csvCoordUnit);

exactTargets = readSphereTargetsFromCsv( ...
    csvFile, csvScale, modelOffset_m, placementOrderMode, startPoint, swapYZColumns);

fprintf('已从 CSV 读取 %d 个小球中心坐标。\n', numel(exactTargets));
fprintf('CSV 文件：%s\n', csvFile);
if swapYZColumns
    fprintf('坐标映射：已交换 y_mm 与 z_mm，即使用 [x_mm, z_mm, y_mm] 作为 [X, Y, Z]。\n');
else
    fprintf('坐标映射：未交换 y_mm 与 z_mm，即使用 [x_mm, y_mm, z_mm] 作为 [X, Y, Z]。\n');
end

rawPts = zeros(numel(exactTargets), 3);
for i = 1:numel(exactTargets)
    rawPts(i,:) = exactTargets(i).originalTargetPoint;
end
rawMinDist = minPairwiseDistance(rawPts);
fprintf('CSV 原始目标点最小球心距 = %.6f m (%.3f mm)\n', rawMinDist, rawMinDist * 1e3);

if rawMinDist < clearance - 1e-12
    warning(['CSV 原始目标点之间已经存在球体重叠：最小球心距 %.3f mm < 要求 %.3f mm。' ...
             '后续网格优化会尝试寻找无重叠可执行点。'], ...
             rawMinDist * 1e3, clearance * 1e3);
end

targets = optimizeTargetsOnGrid(exactTargets, startPoint, gridBase, clearance);
allTargetPts = zeros(numel(targets), 3);
for i = 1:numel(targets)
    allTargetPts(i,:) = targets(i).targetPoint;
end

minDist = minPairwiseDistance(allTargetPts);
fprintf('优化吸附后目标点最小球心距 = %.6f m (%.3f mm)\n', minDist, minDist * 1e3);

if minDist < clearance - 1e-12
    error('优化后的目标点之间仍有重叠，请检查目标优化函数、球半径或安全间隙设置。');
end

supportObjects = [];
if enableSupportObjects
    supportObjects = buildSupportObjectsFromTargets(targets, supportConfig, sphereR, rod);
    fprintf('已生成 %d 个局部辅助支撑物，用于 globalID = %s。\n', ...
        numel(supportObjects), mat2str(supportConfig.targetGlobalIDs));
end
plannerOptions.enableSupportObjects = enableSupportObjects;
plannerOptions.supportObjects = supportObjects;

placedCenters = [];
placedPointIndices = [];
placedGlobalIDs = [];
points = struct([]);

for i = 1:numel(targets)

    thisTarget = targets(i).targetPoint;
    thisExact  = targets(i).originalTargetPoint;
    fprintf('开始规划 %d / %d | globalID=%g | target = [%.3f %.3f %.3f] mm\n', ...
        i, numel(targets), targets(i).globalID, ...
        thisTarget(1)*1e3, thisTarget(2)*1e3, thisTarget(3)*1e3);
    drawnow;

    if ~isempty(placedCenters)
        dists = sqrt(sum((placedCenters - thisTarget).^2, 2));
        if any(dists < clearance - 1e-12)
            error('第 %d 个目标点与已放置小球重叠，无法放置。', i);
        end
    end

    [path, routeType, stepSummary, approachInfo] = planPathAvoidingObstacles( ...
        startPoint, thisTarget, placedCenters, ...
        sphereR, clearance, allowedSteps, gridBase, ...
        plannerOptions, placedPointIndices, placedGlobalIDs);

    points(i).pointIndex = targets(i).pointIndex;
    points(i).label = targets(i).label;
    points(i).targetName = targets(i).targetName;

    points(i).globalID = targets(i).globalID;
    points(i).zLayerID = targets(i).zLayerID;
    points(i).orderInLayer = targets(i).orderInLayer;
    points(i).thetaDeg = targets(i).thetaDeg;
    points(i).sourceClass = targets(i).sourceClass;
    points(i).sourceCurves = targets(i).sourceCurves;

    points(i).routeType = routeType;
    points(i).motionMode = motionMode;
    points(i).finalApproachUsed = approachInfo.finalApproachUsed;
    points(i).approachParentIndex = approachInfo.parentIndex;
    points(i).approachParentGlobalID = approachInfo.parentGlobalID;
    points(i).approachParentDistance_m = approachInfo.parentDistance;
    points(i).approachDirection = roundN(approachInfo.approachDirection, 8);
    points(i).preApproachPoint = roundN(approachInfo.preApproachPoint, pathOutputDecimals);
    points(i).preApproachPoint_mm = roundN(approachInfo.preApproachPoint * 1e3, 4);
    points(i).globalRouteType = approachInfo.globalRouteType;
    points(i).finalApproachRouteType = approachInfo.finalApproachRouteType;
    points(i).finalApproachFailReason = approachInfo.failReason;
    points(i).rodObstacleEnabled = approachInfo.rodObstacleEnabled;
    points(i).targetAboveRodRegion = approachInfo.targetAboveRodRegion;
    points(i).rodSafeY_m = approachInfo.rodSafeY;
    points(i).rodSafeY_mm = approachInfo.rodSafeY * 1e3;
    points(i).rodConstraintChecked = approachInfo.rodConstraintChecked;
    points(i).rodCrossingUsed = approachInfo.rodCrossingUsed;
    points(i).rodRouteType = approachInfo.rodRouteType;
    points(i).rodConstraintFailReason = approachInfo.rodFailReason;
    points(i).startHorizontalConstraintEnabled = approachInfo.startHorizontalConstraintEnabled;
    points(i).startHorizontalUsed = approachInfo.startHorizontalUsed;
    points(i).startHorizontalDistance_m = approachInfo.startHorizontalDistance;
    points(i).startHorizontalDistance_mm = approachInfo.startHorizontalDistance * 1e3;
    points(i).startHorizontalExitPoint = roundN(approachInfo.startHorizontalExitPoint, pathOutputDecimals);
    points(i).startHorizontalExitPoint_mm = roundN(approachInfo.startHorizontalExitPoint * 1e3, 4);
    points(i).startHorizontalRouteType = approachInfo.startHorizontalRouteType;
    points(i).startHorizontalFailReason = approachInfo.startHorizontalFailReason;
    points(i).supportObjectsEnabled = approachInfo.supportObjectsEnabled;
    points(i).supportContactAllowed = approachInfo.supportContactAllowed;
    points(i).supportContactObjectNames = approachInfo.supportContactObjectNames;
    points(i).supportCollisionChecked = approachInfo.supportCollisionChecked;
    points(i).supportConstraintFailReason = approachInfo.supportFailReason;

    points(i).csvOriginalPoint_mm = roundN(targets(i).csvOriginalPoint_mm, 6);
    points(i).csvMappedPoint_mm = roundN(targets(i).csvMappedPoint_mm, 6);
    points(i).originalTargetPoint = roundN(thisExact, 7);
    points(i).originalTargetPoint_mm = roundN(thisExact * 1e3, 6);
    points(i).targetPoint = roundN(thisTarget, 4);
    points(i).targetPoint_mm = roundN(thisTarget * 1e3, 3);

    points(i).path = roundN(path, pathOutputDecimals);
    points(i).finalApproachPath = roundN(approachInfo.finalApproachPath, pathOutputDecimals);
    points(i).stepSummary = stepSummary;

    placedCenters = [placedCenters; thisTarget]; %#ok<AGROW>
    placedPointIndices = [placedPointIndices; targets(i).pointIndex]; %#ok<AGROW>
    placedGlobalIDs = [placedGlobalIDs; targets(i).globalID]; %#ok<AGROW>

    if mod(i, 10) == 0 || i == numel(targets)
        fprintf('已完成 %d / %d 个目标点路径规划。\n', i, numel(targets));
    end
end

root.description = ['从 CSV 读取小球中心坐标的逐点堆积避碰路径规划；' ...
    'CSV 坐标列为 x_mm / y_mm / z_mm；' ...
    '可选在读取后交换 y_mm 与 z_mm 两列再作为目标坐标；' ...
    '程序内部统一转换为 m；' ...
    '所有点均从同一起点出发；' ...
    '已放置小球按半径 1 mm 球形障碍物处理；' ...
    '可选 axis_only 单轴运动或 multi_axis 多轴联动运动；' ...
    'multi_axis 模式下可选最后正面贴近目标球；' ...
    '规则路径失败后可自动 A* 搜索；' ...
    '每个小球从起始点出发后，先保持高度不变水平前进 4 mm，再允许上下移动；' ...
    '目标点先在 0.1 mm 网格上优化吸附，避免目标球重叠；' ...
    '可选固定长杆障碍物及其正上方恒定高度水平跨越约束。'];

root.inputCsvFile = csvFile;
root.csvCoordinateUnit = csvCoordUnit;
root.swapYZColumns = swapYZColumns;
if swapYZColumns
    root.coordinateMapping = '[x_mm, z_mm, y_mm] -> [X, Y, Z]';
else
    root.coordinateMapping = '[x_mm, y_mm, z_mm] -> [X, Y, Z]';
end
root.modelOffset_m = roundN(modelOffset_m, 7);
root.centerPoint = roundN(startPoint, 4);
root.sphereRadius_m = sphereR;
root.clearance_m = clearance;
root.allowedSteps_m = allowedSteps;
root.gridBase_m = gridBase;
root.motionMode = motionMode;
root.enableFinalApproach = enableFinalApproach;
root.approachParentMode = approachParentMode;
root.contactTolerance_m = contactTolerance;
root.finalApproachDistance_m = finalApproachDistance;
root.finalApproachStep_m = finalApproachStep;
root.multiAxisPathStep_m = multiAxisPathStep;
root.finalApproachFailMode = finalApproachFailMode;
root.enableMultiAxisAstar = enableMultiAxisAstar;
root.pathOutputDecimals = pathOutputDecimals;
root.enableRodObstacle = enableRodObstacle;
root.rodObstacle = rod;
root.rodRule = '路径经过长杆正上方区域时，Y 方向高度必须保持不变；普通小球还必须满足 safeY 高度，X/Z 方向水平移动允许。';
root.enableSupportObjects = enableSupportObjects;
root.supportObjects = supportObjects;
root.supportObjectRule = ['局部短托杆/小托架仅作为局部支撑障碍物参与碰撞检测；' ...
    '当前采用方案A的紧凑小托盘尺寸，减小Z向占用以避免干涉相邻轮廓点；' ...
    '不施加 safeY，也不施加正上方 Y 不变规则；' ...
    '对指定被支撑小球允许最终近接触。'];
root.enableStartHorizontalConstraint = enableStartHorizontalConstraint;
root.startHorizontalConstraint = startConstraint;
root.startHorizontalRule = '每个小球从起始点出发后，必须先保持 Y 高度不变，沿水平方向前进 4 mm，之后才允许上下移动。';
root.placementOrderMode = placementOrderMode;
root.rawMinTargetDistance_m = rawMinDist;
root.optimizedMinTargetDistance_m = minDist;
% root.pointCount = numel(points);
% root.points = points;
root.pointCount = numel(points);
root.points = structArrayToCellArray(points);
allFile = fullfile(outFolder, 'csv_all_points_paths_obstacle.json');
writeJsonFile(allFile, root);

for i = 1:numel(points)
    single.description = 'CSV 小球结构单点避碰路径规划文件';
    single.inputCsvFile = csvFile;
    single.swapYZColumns = swapYZColumns;
    if swapYZColumns
        single.coordinateMapping = '[x_mm, z_mm, y_mm] -> [X, Y, Z]';
    else
        single.coordinateMapping = '[x_mm, y_mm, z_mm] -> [X, Y, Z]';
    end
    single.centerPoint = roundN(startPoint, 4);
    single.sphereRadius_m = sphereR;
    single.clearance_m = clearance;
    single.motionMode = motionMode;
    single.enableFinalApproach = enableFinalApproach;
    single.approachParentMode = approachParentMode;
    single.finalApproachDistance_m = finalApproachDistance;
    single.multiAxisPathStep_m = multiAxisPathStep;
    single.enableRodObstacle = enableRodObstacle;
    single.rodObstacle = rod;
    single.enableSupportObjects = enableSupportObjects;
    single.supportObjects = supportObjects;
    single.enableStartHorizontalConstraint = enableStartHorizontalConstraint;
    single.startHorizontalConstraint = startConstraint;
    % single.points = points(i);
    single.points = {points(i)};
    fileName = sprintf('csv_point_%03d_obstacle.json', points(i).pointIndex);
    filePath = fullfile(outFolder, fileName);
    writeJsonFile(filePath, single);
end

fprintf('---------------------------------------------\n');
fprintf('路径规划完成，共 %d 个目标点。\n', numel(points));
fprintf('总汇文件：%s\n', allFile);
fprintf('单点文件夹：%s\n', outFolder);
fprintf('---------------------------------------------\n');

for i = 1:min(10, numel(points))
    fprintf('点%03d | %s | 路径类型: %s | 总步数: %d\n', ...
        points(i).pointIndex, points(i).targetName, ...
        points(i).routeType, points(i).stepSummary.totalSteps);
end

if doPlot
    figure('Color','w'); hold on; axis equal; grid on; view(3);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('CSV小球结构逐点堆积避碰路径规划');

    [sx, sy, sz] = sphere(18);

    for i = 1:numel(targets)
        c = targets(i).targetPoint;
        surf(sphereR*sx + c(1), sphereR*sy + c(2), sphereR*sz + c(3), ...
            'EdgeColor', 'none', 'FaceAlpha', 0.16);

        if showPointLabels
            text(c(1), c(2), c(3), sprintf(' %d', i), 'FontSize', 7);
        end
    end

    scatter3(startPoint(1), startPoint(2), startPoint(3), 60, 'k', 'filled');
    text(startPoint(1), startPoint(2), startPoint(3), ' start', 'FontWeight', 'bold');

    % if enableRodObstacle
    %     drawRodObstacle(rod);
    % end

    % if enableSupportObjects && ~isempty(supportObjects)
    %     drawSupportObjects(supportObjects);
    % end

    if plotAllPaths
        nPlot = min(numel(points), maxPathsToPlot);
        for i = 1:nPlot
            p = points(i).path;
            plot3(p(:,1), p(:,2), p(:,3), 'LineWidth', 0.7);

            fp = points(i).finalApproachPath;
            if ~isempty(fp) && size(fp,1) >= 2
                plot3(fp(:,1), fp(:,2), fp(:,3), 'LineWidth', 2.0);
                pp = points(i).preApproachPoint;
                if all(isfinite(pp))
                    scatter3(pp(1), pp(2), pp(3), 18, 'filled');
                end
            end
        end
    end

    camlight headlight;
    lighting gouraud;
end

function scale = unitToMeterScale(unitName)
    switch lower(strtrim(unitName))
        case 'm'
            scale = 1;
        case 'mm'
            scale = 1e-3;
        case 'um'
            scale = 1e-6;
        case 'micron'
            scale = 1e-6;
        otherwise
            error('不支持的坐标单位：%s。请使用 m / mm / um。', unitName);
    end
end

function exactTargets = readSphereTargetsFromCsv(csvFile, csvScale, modelOffset_m, placementOrderMode, startPoint, swapYZColumns)
    try
        T = readtable(csvFile, 'VariableNamingRule', 'preserve');
    catch
        T = readtable(csvFile);
    end

    if height(T) < 1
        error('CSV 文件为空，未读取到任何小球坐标。');
    end

    varNames = T.Properties.VariableNames;
    xVar = findVarName(varNames, {'x_mm', 'x', 'X_mm', 'X'});
    yVar = findVarName(varNames, {'y_mm', 'y', 'Y_mm', 'Y'});
    zVar = findVarName(varNames, {'z_mm', 'z', 'Z_mm', 'Z'});
    x = getNumericColumn(T, xVar);
    y = getNumericColumn(T, yVar);
    z = getNumericColumn(T, zVar);

    if any(isnan(x)) || any(isnan(y)) || any(isnan(z))
        error('CSV 的 x/y/z 坐标中存在 NaN 或非数值内容，请检查文件。');
    end

    P_csv_original = [x, y, z];

    % false: [x, y, z] -> [X, Y, Z]
    % true : [x, z, y] -> [X, Y, Z]
    if swapYZColumns
        tmp = y;
        y = z;
        z = tmp;
    end

    P_mm_or_raw = [x, y, z];
    P_m = P_mm_or_raw * csvScale + modelOffset_m;
    order = getPlacementOrder(T, P_m, placementOrderMode, startPoint);
    T = T(order, :);
    P_csv_original = P_csv_original(order, :);
    P_mm_or_raw = P_mm_or_raw(order, :);
    P_m = P_m(order, :);
    n = size(P_m, 1);
    exactTargets = struct([]);

    for i = 1:n
        exactTargets(i).pointIndex = i; %#ok<AGROW>
        exactTargets(i).label = sprintf('第%d个点移动', i);

        globalID = getOptionalNumericValue(T, 'global_id', i, i);
        zLayerID = getOptionalNumericValue(T, 'z_layer_id', i, NaN);
        orderInLayer = getOptionalNumericValue(T, 'order_in_layer', i, NaN);
        thetaDeg = getOptionalNumericValue(T, 'theta_deg', i, NaN);
        sourceClass = getOptionalTextValue(T, 'source_class', i, '');
        sourceCurves = getOptionalTextValue(T, 'source_curves', i, '');

        exactTargets(i).targetName = sprintf('csv_global_%g', globalID);
        exactTargets(i).globalID = globalID;
        exactTargets(i).zLayerID = zLayerID;
        exactTargets(i).orderInLayer = orderInLayer;
        exactTargets(i).thetaDeg = thetaDeg;
        exactTargets(i).sourceClass = sourceClass;
        exactTargets(i).sourceCurves = sourceCurves;
        exactTargets(i).csvOriginalPoint_mm = P_csv_original(i,:);
        exactTargets(i).csvMappedPoint_mm = P_mm_or_raw(i,:);
        exactTargets(i).originalTargetPoint = P_m(i,:);
    end
end

function name = findVarName(varNames, candidates)
    normNames = normalizeNames(varNames);

    for i = 1:numel(candidates)
        cand = normalizeOneName(candidates{i});
        hit = find(strcmp(normNames, cand), 1);
        if ~isempty(hit)
            name = varNames{hit};
            return;
        end
    end

    error('CSV 中未找到所需坐标列。候选列名包括：%s', strjoin(candidates, ', '));
end

function normNames = normalizeNames(names)
    normNames = cell(size(names));
    for i = 1:numel(names)
        normNames{i} = normalizeOneName(names{i});
    end
end

function s = normalizeOneName(s)
    s = char(s);
    s = lower(strtrim(s));
    s = strrep(s, ' ', '');
    s = strrep(s, '-', '_');
end

function col = getNumericColumn(T, varName)
    v = T.(varName);

    if isnumeric(v)
        col = double(v);
    elseif iscell(v)
        col = nan(numel(v), 1);
        for i = 1:numel(v)
            col(i) = str2double(v{i});
        end
    elseif isstring(v)
        col = str2double(v);
    else
        try
            col = str2double(string(v));
        catch
            error('列 %s 无法转换为数值。', varName);
        end
    end

    col = col(:);
end

function order = getPlacementOrder(T, P_m, placementOrderMode, startPoint)
    n = size(P_m, 1);

    switch lower(strtrim(placementOrderMode))
        case 'csv'

            if hasVar(T, 'global_id')
                gid = getNumericColumn(T, getActualVarName(T, 'global_id'));
                [~, order] = sort(gid, 'ascend');
            else
                order = (1:n)';
            end

        case 'auto_closure_risk'

            if hasVar(T, 'global_id')
                gid = getNumericColumn(T, getActualVarName(T, 'global_id'));
                [~, baseOrder] = sort(gid, 'ascend');
            else
                baseOrder = (1:n)';
            end
            order = applyAutoClosureRiskOrder(T, P_m, baseOrder);

        case 'z_layer_then_order'
            if hasVar(T, 'z_layer_id') && hasVar(T, 'order_in_layer')
                zLayer = getNumericColumn(T, getActualVarName(T, 'z_layer_id'));
                inLayer = getNumericColumn(T, getActualVarName(T, 'order_in_layer'));
                [~, order] = sortrows([zLayer, inLayer], [1, 2]);
            else
                error('placementOrderMode = z_layer_then_order 时，CSV 需要包含 z_layer_id 和 order_in_layer 两列。');
            end

        case 'distance_to_start'
            d = sqrt(sum((P_m - startPoint).^2, 2));
            [~, order] = sort(d, 'ascend');

        otherwise
            error(['未知 placementOrderMode：%s。可选 csv / z_layer_then_order / ' ...
                   'distance_to_start / auto_closure_risk。'], placementOrderMode);
    end

    order = order(:);
end

function order = applyAutoClosureRiskOrder(T, P_m, baseOrder)
    n = size(P_m, 1);
    order = baseOrder(:)';

    if n <= 1
        order = order(:);
        return;
    end

    if hasVar(T, 'global_id')
        gid = getNumericColumn(T, getActualVarName(T, 'global_id'));
    else
        gid = (1:n)';
    end

    if hasVar(T, 'z_layer_id')
        zLayer = getNumericColumn(T, getActualVarName(T, 'z_layer_id'));
    else
        zLayer = ones(n,1);
    end
    maxLayer = max(zLayer(~isnan(zLayer)));
    sourceClass = repmat({''}, n, 1);
    sourceCurves = repmat({''}, n, 1);
    for ii = 1:n
        sourceClass{ii} = lower(strtrim(getOptionalTextValue(T, 'source_class', ii, '')));
        sourceCurves{ii} = char(getOptionalTextValue(T, 'source_curves', ii, ''));
    end

    neighborRadius = 2.35e-3;
    minNeighborCount = 3;
    D = squareformDistance(P_m);
    riskRows = [];
    supportRows = [];

    for row = 1:n
        curveText = sourceCurves{row};
        curveTextLower = lower(curveText);
        isMixedPoint = strcmp(sourceClass{row}, 'mixed') || contains(curveText, '|');
        if ~isMixedPoint
            continue;
        end

        neighborRows = find(D(row,:) < neighborRadius & D(row,:) > 1e-12);
        if numel(neighborRows) < minNeighborCount
            continue;
        end

        isTopClosurePoint = contains(curveTextLower, 'toploop') || zLayer(row) == maxLayer;
        if ~isTopClosurePoint
            continue;
        end

        bridgeNames = extractBridgeCurveNames(curveText);
        if isempty(bridgeNames)
            continue;
        end

        support = findBridgeSupportNeighbor(row, neighborRows, sourceCurves, bridgeNames, gid);
        if isnan(support)
            continue;
        end

        riskRows(end+1) = row; %#ok<AGROW>
        supportRows(end+1) = support; %#ok<AGROW>
    end

    if isempty(riskRows)
        return;
    end

    supportPos = zeros(numel(supportRows),1);
    for k = 1:numel(supportRows)
        pos = find(order == supportRows(k), 1, 'first');
        if isempty(pos)
            pos = inf;
        end
        supportPos(k) = pos;
    end
    [~, ord] = sort(supportPos, 'ascend');
    riskRows = riskRows(ord);
    supportRows = supportRows(ord);

    movedMsg = {};

    for k = 1:numel(riskRows)
        risk = riskRows(k);
        support = supportRows(k);

        posRisk = find(order == risk, 1, 'first');
        posSupport = find(order == support, 1, 'first');
        if isempty(posRisk) || isempty(posSupport)
            continue;
        end
        if posRisk == posSupport + 1
            continue;
        end

        order(order == risk) = [];
        posSupport = find(order == support, 1, 'first');
        order = [order(1:posSupport), risk, order(posSupport+1:end)]; %#ok<AGROW>

        movedMsg{end+1} = sprintf('globalID %.0f -> after globalID %.0f', gid(risk), gid(support)); %#ok<AGROW>
    end

    if ~isempty(movedMsg)
        fprintf('自动闭合风险点重排序：%s\n', strjoin(movedMsg, '; '));
    end

    order = order(:);
end

function D = squareformDistance(P)
    n = size(P,1);
    D = zeros(n,n);
    for ii = 1:n-1
        diff = P(ii+1:end,:) - P(ii,:);
        d = sqrt(sum(diff.^2, 2));
        D(ii, ii+1:end) = d;
        D(ii+1:end, ii) = d;
    end
end

function bridgeNames = extractBridgeCurveNames(curveText)
    parts = strsplit(char(curveText), '|');
    bridgeNames = {};
    for ii = 1:numel(parts)
        s = strtrim(parts{ii});
        if contains(lower(s), 'bridge')
            bridgeNames{end+1} = s; %#ok<AGROW>
        end
    end
end

function support = findBridgeSupportNeighbor(row, neighborRows, sourceCurves, bridgeNames, gid)
    support = NaN;
    candidates = [];

    for kk = 1:numel(neighborRows)
        nb = neighborRows(kk);
        nbText = lower(sourceCurves{nb});
        hasSameBridge = false;
        for jj = 1:numel(bridgeNames)
            if contains(nbText, lower(bridgeNames{jj}))
                hasSameBridge = true;
                break;
            end
        end
        if hasSameBridge && gid(nb) < gid(row)
            candidates(end+1) = nb; %#ok<AGROW>
        end
    end

    if isempty(candidates)
        return;
    end

    [~, idx] = max(gid(candidates));
    support = candidates(idx);
end

function tf = hasVar(T, name)
    names = T.Properties.VariableNames;
    tf = any(strcmp(normalizeNames(names), normalizeOneName(name)));
end

function actualName = getActualVarName(T, name)
    names = T.Properties.VariableNames;
    normNames = normalizeNames(names);
    hit = find(strcmp(normNames, normalizeOneName(name)), 1);
    if isempty(hit)
        error('未找到列：%s', name);
    end
    actualName = names{hit};
end

function val = getOptionalNumericValue(T, varName, rowIdx, defaultVal)
    if hasVar(T, varName)
        actualName = getActualVarName(T, varName);
        col = getNumericColumn(T, actualName);
        val = col(rowIdx);
        if isnan(val)
            val = defaultVal;
        end
    else
        val = defaultVal;
    end
end

function val = getOptionalTextValue(T, varName, rowIdx, defaultVal)
    if hasVar(T, varName)
        actualName = getActualVarName(T, varName);
        v = T.(actualName);
        if iscell(v)
            val = v{rowIdx};
        elseif isstring(v)
            val = char(v(rowIdx));
        elseif isnumeric(v)
            val = num2str(v(rowIdx));
        else
            val = char(string(v(rowIdx)));
        end
    else
        val = defaultVal;
    end
end

function targets = optimizeTargetsOnGrid(exactTargets, startPoint, gridBase, clearance)
    targets = exactTargets;
    placed = [];
    searchRadii = 0:5;
    backupRadii = 6:20;

    for i = 1:numel(exactTargets)
        exactPt = exactTargets(i).originalTargetPoint;
        found = false;
        bestPt = [];

        for rr = [searchRadii, backupRadii]
            candidates = generateGridCandidatesNearPoint(exactPt, startPoint, gridBase, rr);
            errs = sqrt(sum((candidates - exactPt).^2, 2));
            [~, ord] = sort(errs, 'ascend');
            candidates = candidates(ord,:);

            for k = 1:size(candidates,1)
                pt = candidates(k,:);

                if isempty(placed)
                    bestPt = pt;
                    found = true;
                    break;
                else
                    d = sqrt(sum((placed - pt).^2, 2));
                    if all(d >= clearance - 1e-12)
                        bestPt = pt;
                        found = true;
                        break;
                    end
                end
            end

            if found
                break;
            end
        end

        if ~found
            error('第 %d 个目标点无法在网格上找到无重叠可执行点，请增大搜索半径或检查球间距。', i);
        end

        targets(i).targetPoint = bestPt;
        placed = [placed; bestPt]; %#ok<AGROW>
    end
end

function candidates = generateGridCandidatesNearPoint(exactPt, startPoint, gridBase, radiusUnit)
    baseIdx = round((exactPt - startPoint) / gridBase);
    tmp = [];
    for dx = -radiusUnit:radiusUnit
        for dy = -radiusUnit:radiusUnit
            for dz = -radiusUnit:radiusUnit
                idx = baseIdx + [dx, dy, dz];
                pt = startPoint + idx * gridBase;
                tmp = [tmp; pt]; %#ok<AGROW>
            end
        end
    end

    relIdx = round((tmp - startPoint) / gridBase);
    [~, ia, ~] = unique(relIdx, 'rows', 'stable');
    candidates = tmp(ia,:);
end

function md = minPairwiseDistance(pts)
    n = size(pts,1);
    md = inf;

    if n <= 1
        md = inf;
        return;
    end

    for i = 1:n-1
        for j = i+1:n
            d = norm(pts(i,:) - pts(j,:));
            if d < md
                md = d;
            end
        end
    end
end

function [bestPath, bestRouteType, bestSummary, approachInfo] = planPathAvoidingObstacles( ...
    startPt, targetPt, obstacles, sphereR, clearance, allowedSteps, gridBase, ...
    plannerOptions, placedPointIndices, placedGlobalIDs)

    if nargin < 9 || isempty(plannerOptions)
        plannerOptions.motionMode = 'axis_only';
        plannerOptions.enableFinalApproach = false;
    end
    if nargin < 10 || isempty(placedPointIndices)
        placedPointIndices = (1:size(obstacles,1))';
    end
    if nargin < 11 || isempty(placedGlobalIDs)
        placedGlobalIDs = placedPointIndices;
    end

    approachInfo = defaultApproachInfo();
    approachInfo.motionMode = plannerOptions.motionMode;
    approachInfo = fillRodInfoForTarget(approachInfo, targetPt, plannerOptions);
    approachInfo = fillStartHorizontalInfo(approachInfo, plannerOptions);
    approachInfo = fillSupportInfoForTarget(approachInfo, targetPt, plannerOptions);

    switch lower(strtrim(plannerOptions.motionMode))
        case 'axis_only'
            [bestPath, bestRouteType, bestSummary] = planAxisOnlyPathAvoidingObstacles( ...
                startPt, targetPt, obstacles, sphereR, clearance, allowedSteps, gridBase, plannerOptions);
            bestSummary.motionMode = 'axis_only';
            approachInfo = updateStartHorizontalInfoFromRoute(approachInfo, bestRouteType, plannerOptions);
            approachInfo.failReason = 'axis_only 模式未启用最后正面贴近';

        case 'multi_axis'
            [bestPath, bestRouteType, bestSummary, approachInfo] = planMultiAxisPathWithOptionalFinalApproach( ...
                startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
                plannerOptions, placedPointIndices, placedGlobalIDs);

        otherwise
            error('未知 motionMode：%s。可选 axis_only / multi_axis。', plannerOptions.motionMode);
    end
end

function info = defaultApproachInfo()
    info.motionMode = '';
    info.finalApproachUsed = false;
    info.parentIndex = NaN;
    info.parentGlobalID = NaN;
    info.parentDistance = NaN;
    info.approachDirection = [NaN, NaN, NaN];
    info.preApproachPoint = [NaN, NaN, NaN];
    info.finalApproachPath = [];
    info.globalRouteType = '';
    info.finalApproachRouteType = '';
    info.failReason = '';
    info.rodObstacleEnabled = false;
    info.targetAboveRodRegion = false;
    info.rodSafeY = NaN;
    info.rodConstraintChecked = false;
    info.rodCrossingUsed = false;
    info.rodRouteType = '';
    info.rodFailReason = '';
    info.startHorizontalConstraintEnabled = false;
    info.startHorizontalUsed = false;
    info.startHorizontalDistance = NaN;
    info.startHorizontalExitPoint = [NaN, NaN, NaN];
    info.startHorizontalRouteType = '';
    info.startHorizontalFailReason = '';
    info.supportObjectsEnabled = false;
    info.supportContactAllowed = false;
    info.supportContactObjectNames = {};
    info.supportCollisionChecked = false;
    info.supportFailReason = '';
end

function [bestPath, bestRouteType, bestSummary, approachInfo] = planMultiAxisPathWithOptionalFinalApproach( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
    plannerOptions, placedPointIndices, placedGlobalIDs)

    approachInfo = defaultApproachInfo();
    approachInfo.motionMode = 'multi_axis';
    approachInfo = fillRodInfoForTarget(approachInfo, targetPt, plannerOptions);
    approachInfo = fillStartHorizontalInfo(approachInfo, plannerOptions);
    approachInfo = fillSupportInfoForTarget(approachInfo, targetPt, plannerOptions);

    rodAboveTarget = isTargetAboveRodForHorizontalEntry(targetPt, plannerOptions);

    useFinal = isfield(plannerOptions, 'enableFinalApproach') && plannerOptions.enableFinalApproach && ~isempty(obstacles) && ~rodAboveTarget;

    if useFinal
        [hasParent, parentCenter, parentIndex, parentGlobalID, parentDist, parentMode] = findApproachParent( ...
            targetPt, obstacles, placedPointIndices, placedGlobalIDs, clearance, plannerOptions);

        if hasParent
            dirVec = targetPt - parentCenter;
            dirNorm = norm(dirVec);

            if dirNorm > 1e-12
                u = dirVec / dirNorm;

                if isRodObstacleEnabled(plannerOptions) && isTargetAboveRodRegion(targetPt, plannerOptions.rod)
                    u2 = u;
                    u2(plannerOptions.rod.heightAxis) = 0;
                    if norm(u2) < 1e-12
                        u2 = [1, 0, 0];
                    else
                        u2 = u2 / norm(u2);
                    end
                    u = u2;
                end
                preApproachPoint = targetPt + plannerOptions.finalApproachDistance * u;
                finalPath = buildLinePath(preApproachPoint, targetPt, plannerOptions.finalApproachStep);

                approachInfo.parentIndex = parentIndex;
                approachInfo.parentGlobalID = parentGlobalID;
                approachInfo.parentDistance = parentDist;
                approachInfo.approachDirection = u;
                approachInfo.preApproachPoint = preApproachPoint;
                approachInfo.finalApproachRouteType = sprintf('front_approach_%s', parentMode);

                if isPathCollisionFree(finalPath, obstacles, clearance, plannerOptions, targetPt)
                    [okGlobal, globalPath, globalRouteType, ~] = planMultiAxisTransportPath( ...
                        startPt, preApproachPoint, obstacles, clearance, allowedSteps, gridBase, plannerOptions);

                    if okGlobal
                        bestPath = concatPaths(globalPath, finalPath);
                        bestRouteType = sprintf('%s + final_front_approach', globalRouteType);
                        bestSummary = summarizeMultiAxisPath(bestPath);
                        bestSummary.globalRouteType = globalRouteType;
                        bestSummary.finalApproachRouteType = approachInfo.finalApproachRouteType;

                        approachInfo.finalApproachUsed = true;
                        approachInfo.finalApproachPath = finalPath;
                        approachInfo.globalRouteType = globalRouteType;
                        approachInfo = updateStartHorizontalInfoFromRoute(approachInfo, globalRouteType, plannerOptions);
                        if contains(lower(globalRouteType), 'rod_safe') || contains(lower(globalRouteType), 'rod_above')
                            approachInfo.rodCrossingUsed = true;
                            approachInfo.rodRouteType = globalRouteType;
                        end
                        approachInfo.failReason = '';
                        return;
                    else
                        approachInfo.failReason = '正面贴近段可行，但 startPoint 到 preApproachPoint 的多轴运输路径失败';
                    end
                else
                    approachInfo.failReason = 'preApproachPoint 到 targetPoint 的最后正面贴近段与已放置球发生碰撞';
                end
            else
                approachInfo.failReason = 'parentCenter 与 targetPt 距离过小，无法定义正面贴近方向';
            end
        else
            approachInfo.failReason = '未找到合适的已放置接触球作为最后正面贴近对象';
        end

        if isfield(plannerOptions, 'finalApproachFailMode') && strcmpi(plannerOptions.finalApproachFailMode, 'error')
            error('最后正面贴近路径规划失败：%s', approachInfo.failReason);
        end
    else
        if isempty(obstacles)
            approachInfo.failReason = '第一个球无已放置接触球，直接使用普通多轴路径';
        else
            approachInfo.failReason = 'enableFinalApproach = false，直接使用普通多轴路径';
        end
    end

    [okNormal, bestPath, bestRouteType, bestSummary] = planMultiAxisTransportPath( ...
        startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions);

    if ~okNormal
        error('普通多轴路径也未找到可行解。最后正面贴近失败原因：%s', approachInfo.failReason);
    end

    bestRouteType = sprintf('%s + no_final_front_approach', bestRouteType);
    bestSummary.finalApproachRouteType = 'not_used';
    approachInfo.globalRouteType = bestRouteType;
    approachInfo = updateStartHorizontalInfoFromRoute(approachInfo, bestRouteType, plannerOptions);
    if contains(lower(bestRouteType), 'rod_safe') || contains(lower(bestRouteType), 'rod_above')
        approachInfo.rodCrossingUsed = true;
        approachInfo.rodRouteType = bestRouteType;
    end
end

function [hasParent, parentCenter, parentIndex, parentGlobalID, parentDist, modeUsed] = findApproachParent( ...
    targetPt, obstacles, placedPointIndices, placedGlobalIDs, clearance, plannerOptions)

    hasParent = false;
    parentCenter = [NaN, NaN, NaN];
    parentIndex = NaN;
    parentGlobalID = NaN;
    parentDist = NaN;
    modeUsed = '';

    if isempty(obstacles)
        return;
    end

    mode = lower(strtrim(plannerOptions.approachParentMode));
    dists = sqrt(sum((obstacles - targetPt).^2, 2));

    switch mode
        case 'previous_placed'
            idx = size(obstacles, 1);
            hasParent = true;
            modeUsed = 'previous_placed';

        case {'nearest_placed_contact', 'auto'}
            tol = plannerOptions.contactTolerance;
            candidates = find(abs(dists - clearance) <= tol);

            if isempty(candidates)
                hasParent = false;
                return;
            end

            err = abs(dists(candidates) - clearance);
            [~, localOrder] = sortrows([err, dists(candidates)], [1, 2]);
            idx = candidates(localOrder(1));
            hasParent = true;
            modeUsed = 'nearest_placed_contact';

        otherwise
            error('未知 approachParentMode：%s。', plannerOptions.approachParentMode);
    end

    parentCenter = obstacles(idx,:);
    parentDist = dists(idx);

    if idx <= numel(placedPointIndices)
        parentIndex = placedPointIndices(idx);
    else
        parentIndex = idx;
    end

    if idx <= numel(placedGlobalIDs)
        parentGlobalID = placedGlobalIDs(idx);
    else
        parentGlobalID = parentIndex;
    end
end

function [ok, bestPath, bestRouteType, bestSummary] = planMultiAxisTransportPath( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];
    stepLen = plannerOptions.multiAxisPathStep;

    if isStartHorizontalConstraintActive(startPt, plannerOptions)
        [okPrefix, prefixPath, exitPt, failReason] = buildStartHorizontalPrefixPath( ...
            startPt, obstacles, clearance, plannerOptions, targetPt);
        if ~okPrefix
            ok = false;
            bestPath = [];
            bestRouteType = sprintf('start_horizontal_failed_%s', failReason);
            bestSummary = [];
            return;
        end

        plannerOptionsRest = disableStartHorizontalConstraint(plannerOptions);
        [okRest, restPath, restRouteType, ~] = planMultiAxisTransportPath( ...
            exitPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptionsRest);

        if okRest
            bestPath = concatPaths(prefixPath, restPath);
            bestRouteType = sprintf('start_horizontal_%.1fmm + %s', ...
                plannerOptions.startConstraint.forwardDistance * 1e3, restRouteType);
            bestSummary = summarizeMultiAxisPath(bestPath);
            bestSummary.startHorizontalUsed = true;
            bestSummary.startHorizontalDistance_m = plannerOptions.startConstraint.forwardDistance;
            ok = true;
            return;
        else
            ok = false;
            bestPath = [];
            bestRouteType = sprintf('start_horizontal_ok_but_rest_failed + %s', restRouteType);
            bestSummary = [];
            return;
        end
    end

    if isempty(obstacles)
        path = buildLinePath(startPt, targetPt, stepLen);
        if isPathCollisionFree(path, obstacles, clearance, plannerOptions, targetPt)
            bestPath = path;
            bestRouteType = 'direct_multi_line_no_obstacle';
            bestSummary = summarizeMultiAxisPath(path);
            ok = true;
            return;
        end
    end

    obsAll = [obstacles; targetPt; startPt];
    xmin = min(obsAll(:,1)); xmax = max(obsAll(:,1));
    ymin = min(obsAll(:,2)); ymax = max(obsAll(:,2));
    zmin = min(obsAll(:,3)); zmax = max(obsAll(:,3));
    margin = clearance + 3 * gridBase;
    xHigh = snapValue(xmax + margin, startPt(1), gridBase);
    xLow  = snapValue(xmin - margin, startPt(1), gridBase);
    yHigh = snapValue(ymax + margin, startPt(2), gridBase);
    yLow  = snapValue(ymin - margin, startPt(2), gridBase);
    zHigh = snapValue(zmax + margin, startPt(3), gridBase);
    zLow  = snapValue(zmin - margin, startPt(3), gridBase);
    candidates = {};

    if isRodObstacleEnabled(plannerOptions)

        rodAboveTargetWpsSet = buildRodAboveTargetHorizontalEntryWaypointCandidates(startPt, targetPt, plannerOptions.rod);
        for kk = 1:numel(rodAboveTargetWpsSet)
            candidates{end+1} = struct('name', sprintf('rod_above_target_horizontal_entry_%02d', kk), ...
                'wps', removeDuplicateWaypoints(startPt, rodAboveTargetWpsSet{kk})); %#ok<AGROW>
        end

        rodWps = buildRodSafeHorizontalCrossWaypoints(startPt, targetPt, plannerOptions.rod);
        if ~isempty(rodWps)
            candidates{end+1} = struct('name', 'rod_safe_horizontal_cross', ...
                'wps', removeDuplicateWaypoints(startPt, rodWps));
        end
    end

    candidates{end+1} = struct('name', 'direct_multi_line', ...
        'wps', targetPt);

    candidates{end+1} = struct('name', 'top_multi_Z', ...
        'wps', removeDuplicateWaypoints(startPt, [
            startPt(1),  startPt(2),  zHigh;
            targetPt(1), targetPt(2), zHigh;
            targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'bottom_multi_Z', ...
        'wps', removeDuplicateWaypoints(startPt, [
            startPt(1),  startPt(2),  zLow;
            targetPt(1), targetPt(2), zLow;
            targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'yHigh_multi', ...
        'wps', removeDuplicateWaypoints(startPt, [
            startPt(1),  yHigh, startPt(3);
            targetPt(1), yHigh, targetPt(3);
            targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'yLow_multi', ...
        'wps', removeDuplicateWaypoints(startPt, [
            startPt(1),  yLow, startPt(3);
            targetPt(1), yLow, targetPt(3);
            targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'xHigh_multi', ...
        'wps', removeDuplicateWaypoints(startPt, [
            xHigh, startPt(2),  startPt(3);
            xHigh, targetPt(2), targetPt(3);
            targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'xLow_multi', ...
        'wps', removeDuplicateWaypoints(startPt, [
            xLow, startPt(2),  startPt(3);
            xLow, targetPt(2), targetPt(3);
            targetPt(1), targetPt(2), targetPt(3)]));

    feasible = {};
    cnt = 0;

    for i = 1:numel(candidates)
        [okRoute, path, summary] = buildMultiAxisRouteAndCheck( ...
            startPt, candidates{i}.wps, obstacles, clearance, stepLen, plannerOptions, targetPt);

        if okRoute
            cnt = cnt + 1;
            feasible{cnt}.name = candidates{i}.name; %#ok<AGROW>
            feasible{cnt}.path = path; %#ok<AGROW>
            feasible{cnt}.summary = summary; %#ok<AGROW>
            feasible{cnt}.priority = i; %#ok<AGROW>
        end
    end

    if ~isempty(feasible)
        bestIdx = 1;
        bestSteps = feasible{1}.summary.totalSteps;
        bestLen = feasible{1}.summary.totalLength_m;
        bestPriority = feasible{1}.priority;

        for i = 2:numel(feasible)
            s = feasible{i}.summary.totalSteps;
            L = feasible{i}.summary.totalLength_m;
            p = feasible{i}.priority;
            if (s < bestSteps) || (s == bestSteps && L < bestLen) || (s == bestSteps && abs(L-bestLen) < 1e-12 && p < bestPriority)
                bestIdx = i;
                bestSteps = s;
                bestLen = L;
                bestPriority = p;
            end
        end

        bestPath = feasible{bestIdx}.path;
        bestRouteType = feasible{bestIdx}.name;
        bestSummary = feasible{bestIdx}.summary;
        ok = true;
        return;
    end

    if isRodObstacleEnabled(plannerOptions) && isTargetAboveRodRegion(targetPt, plannerOptions.rod)
        fprintf('    规则路径失败，尝试 rod-above 2D X-Z A* 水平绕障。\n');
        drawnow;
        [ok2D, path2D, route2D, summary2D] = planRodAboveTarget2DXZPath( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions);
        if ok2D
            bestPath = path2D;
            bestRouteType = route2D;
            bestSummary = summary2D;
            ok = true;
            return;
        end
    end

    if isfield(plannerOptions, 'enableMultiAxisAstar') && plannerOptions.enableMultiAxisAstar
        searchPads = [0.003, 0.006, 0.010, 0.015, 0.020, 0.030];
        for i = 1:numel(searchPads)
            fprintf('    规则路径失败，尝试 multi-axis A*：pad = %.1f mm\n', searchPads(i)*1e3);
            drawnow;
            [okAstar, pathAstar] = aStarMultiAxisSearch( ...
                startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, searchPads(i), plannerOptions);

            if okAstar

                if norm(pathAstar(end,:) - targetPt) > 1e-12
                    tailPath = buildLinePath(pathAstar(end,:), targetPt, stepLen);
                    if ~isPathCollisionFree(tailPath, obstacles, clearance, plannerOptions, targetPt)
                        continue;
                    end
                    pathAstar = concatPaths(pathAstar, tailPath);
                end

                bestPath = pathAstar;
                bestRouteType = sprintf('multi_astar26_pad_%0.1fmm', searchPads(i)*1e3);
                bestSummary = summarizeMultiAxisPath(pathAstar);
                ok = true;
                return;
            end
        end
    end
end

function [ok, path, summary] = buildMultiAxisRouteAndCheck(startPt, waypoints, obstacles, clearance, maxStepLen, plannerOptions, targetPt)
    curr = startPt;
    path = curr;
    ok = true;

    for i = 1:size(waypoints,1)
        wp = waypoints(i,:);
        segPath = buildLinePath(curr, wp, maxStepLen);

        if ~isPathCollisionFree(segPath, obstacles, clearance, plannerOptions, targetPt)
            ok = false;
            path = [];
            summary = [];
            return;
        end

        path = concatPaths(path, segPath);
        curr = wp;
    end

    summary = summarizeMultiAxisPath(path);
end

function path = buildLinePath(p0, p1, maxStepLen)
    dist = norm(p1 - p0);

    if dist < 1e-12
        path = p0;
        return;
    end

    nSteps = max(1, ceil(dist / maxStepLen));
    t = linspace(0, 1, nSteps + 1)';
    path = p0 + t .* (p1 - p0);
end

function ok = isPathCollisionFree(path, obstacles, clearance, plannerOptions, targetPt)
    ok = true;

    if nargin < 4
        plannerOptions = struct();
    end
    if nargin < 5
        targetPt = [NaN, NaN, NaN];
    end

    if size(path,1) <= 1
        return;
    end

    for i = 2:size(path,1)
        p0 = path(i-1,:);
        p1 = path(i,:);

        if segmentCollidesWithAnySphere(p0, p1, obstacles, clearance)
            ok = false;
            return;
        end

        if ~segmentIsValidWithRod(p0, p1, plannerOptions, targetPt)
            ok = false;
            return;
        end

        if ~segmentIsValidWithSupportObjects(p0, p1, plannerOptions, targetPt)
            ok = false;
            return;
        end
    end
end

function pathOut = concatPaths(pathA, pathB)
    if isempty(pathA)
        pathOut = pathB;
        return;
    end
    if isempty(pathB)
        pathOut = pathA;
        return;
    end

    if norm(pathA(end,:) - pathB(1,:)) < 1e-12
        pathOut = [pathA; pathB(2:end,:)];
    else
        pathOut = [pathA; pathB];
    end
end

function summary = summarizeMultiAxisPath(path)
    summary.motionMode = 'multi_axis';
    summary.routeAxes = {};
    summary.step_0p5_mm = 0;
    summary.step_0p2_mm = 0;
    summary.step_0p1_mm = 0;
    summary.singleAxisSteps = 0;
    summary.dualAxisSteps = 0;
    summary.tripleAxisSteps = 0;
    summary.totalSteps = max(0, size(path,1) - 1);
    summary.turnCount = 0;
    summary.totalLength_m = 0;
    summary.meanStepLength_m = 0;
    summary.maxStepLength_m = 0;
    summary.minStepLength_m = 0;
    summary.globalRouteType = '';
    summary.finalApproachRouteType = '';

    if size(path,1) <= 1
        return;
    end

    stepLens = zeros(size(path,1)-1, 1);
    lastDir = [];

    for i = 2:size(path,1)
        dp = path(i,:) - path(i-1,:);
        st = norm(dp);
        stepLens(i-1) = st;
        summary.totalLength_m = summary.totalLength_m + st;
        axesCount = sum(abs(dp) > 1e-12);
        if axesCount == 1
            summary.singleAxisSteps = summary.singleAxisSteps + 1;
        elseif axesCount == 2
            summary.dualAxisSteps = summary.dualAxisSteps + 1;
        elseif axesCount == 3
            summary.tripleAxisSteps = summary.tripleAxisSteps + 1;
        end

        summary.routeAxes{end+1} = movementAxisLabel(dp); %#ok<AGROW>

        if abs(st - 0.0005) < 1e-8
            summary.step_0p5_mm = summary.step_0p5_mm + 1;
        elseif abs(st - 0.0002) < 1e-8
            summary.step_0p2_mm = summary.step_0p2_mm + 1;
        elseif abs(st - 0.0001) < 1e-8
            summary.step_0p1_mm = summary.step_0p1_mm + 1;
        end

        if st > 1e-12
            currDir = dp / st;
            if ~isempty(lastDir)
                if norm(currDir - lastDir) > 1e-6
                    summary.turnCount = summary.turnCount + 1;
                end
            end
            lastDir = currDir;
        end
    end

    summary.meanStepLength_m = mean(stepLens);
    summary.maxStepLength_m = max(stepLens);
    summary.minStepLength_m = min(stepLens);
end

function label = movementAxisLabel(dp)
    names = {'X','Y','Z'};
    idx = find(abs(dp) > 1e-12);

    if isempty(idx)
        label = 'none';
        return;
    end

    label = '';
    for i = 1:numel(idx)
        label = [label, names{idx(i)}]; %#ok<AGROW>
    end
end

function [bestPath, bestRouteType, bestSummary] = planAxisOnlyPathAvoidingObstacles( ...
    startPt, targetPt, obstacles, sphereR, clearance, allowedSteps, gridBase, plannerOptions)

    %#ok<NASGU>

    if isStartHorizontalConstraintActive(startPt, plannerOptions)
        [okPrefix, prefixPath, exitPt, failReason] = buildStartHorizontalPrefixPath( ...
            startPt, obstacles, clearance, plannerOptions, targetPt);
        if ~okPrefix
            error('起始点水平前进约束路径失败：%s', failReason);
        end
        plannerOptionsRest = disableStartHorizontalConstraint(plannerOptions);
        [restPath, restRouteType, ~] = planAxisOnlyPathAvoidingObstacles( ...
            exitPt, targetPt, obstacles, sphereR, clearance, allowedSteps, gridBase, plannerOptionsRest);
        bestPath = concatPaths(prefixPath, restPath);
        bestRouteType = sprintf('start_horizontal_%.1fmm + %s', ...
            plannerOptions.startConstraint.forwardDistance * 1e3, restRouteType);
        bestSummary = summarizeAxisPath(bestPath);
        bestSummary.startHorizontalUsed = true;
        bestSummary.startHorizontalDistance_m = plannerOptions.startConstraint.forwardDistance;
        return;
    end

    if isempty(obstacles)
        wps = buildDirectWaypointsXZY(startPt, targetPt);
        [ok, path, summary] = buildRouteAndCheck(startPt, wps, obstacles, ...
            clearance, allowedSteps, plannerOptions, targetPt);

        if ok
            bestPath = path;
            bestRouteType = 'direct_XZY';
            bestSummary = summary;
            return;
        end
    end

    obsAll = [obstacles; targetPt; startPt];
    xmin = min(obsAll(:,1)); xmax = max(obsAll(:,1));
    ymin = min(obsAll(:,2)); ymax = max(obsAll(:,2));
    zmin = min(obsAll(:,3)); zmax = max(obsAll(:,3));
    margin = clearance + 3 * gridBase;
    xHigh = snapValue(xmax + margin, startPt(1), gridBase);
    xLow  = snapValue(xmin - margin, startPt(1), gridBase);
    yHigh = snapValue(ymax + margin, startPt(2), gridBase);
    yLow  = snapValue(ymin - margin, startPt(2), gridBase);
    zHigh = snapValue(zmax + margin, startPt(3), gridBase);
    zLow  = snapValue(zmin - margin, startPt(3), gridBase);
    candidates = {};

    candidates{end+1} = struct('name', 'direct_XZY', 'wps', buildDirectWaypointsXZY(startPt, targetPt));

    candidates{end+1} = struct('name', 'top_ZXYZ', 'wps', removeDuplicateWaypoints(startPt, [
        startPt(1),  startPt(2),  zHigh;
        targetPt(1), startPt(2),  zHigh;
        targetPt(1), targetPt(2), zHigh;
        targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'bottom_ZXYZ', 'wps', removeDuplicateWaypoints(startPt, [
        startPt(1),  startPt(2),  zLow;
        targetPt(1), startPt(2),  zLow;
        targetPt(1), targetPt(2), zLow;
        targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'yHigh_YXZY', 'wps', removeDuplicateWaypoints(startPt, [
        startPt(1),  yHigh, startPt(3);
        targetPt(1), yHigh, startPt(3);
        targetPt(1), yHigh, targetPt(3);
        targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'yLow_YXZY', 'wps', removeDuplicateWaypoints(startPt, [
        startPt(1),  yLow, startPt(3);
        targetPt(1), yLow, startPt(3);
        targetPt(1), yLow, targetPt(3);
        targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'xHigh_XYZX', 'wps', removeDuplicateWaypoints(startPt, [
        xHigh, startPt(2),  startPt(3);
        xHigh, targetPt(2), startPt(3);
        xHigh, targetPt(2), targetPt(3);
        targetPt(1), targetPt(2), targetPt(3)]));

    candidates{end+1} = struct('name', 'xLow_XYZX', 'wps', removeDuplicateWaypoints(startPt, [
        xLow, startPt(2),  startPt(3);
        xLow, targetPt(2), startPt(3);
        xLow, targetPt(2), targetPt(3);
        targetPt(1), targetPt(2), targetPt(3)]));

    feasible = {};
    cnt = 0;

    for i = 1:numel(candidates)
        [ok, path, summary] = buildRouteAndCheck( ...
            startPt, candidates{i}.wps, obstacles, clearance, allowedSteps, plannerOptions, targetPt);

        if ok
            cnt = cnt + 1;
            feasible{cnt}.name = candidates{i}.name; %#ok<AGROW>
            feasible{cnt}.path = path; %#ok<AGROW>
            feasible{cnt}.summary = summary; %#ok<AGROW>
            feasible{cnt}.priority = i; %#ok<AGROW>
        end
    end

    if ~isempty(feasible)
        bestIdx = 1;
        bestSteps = feasible{1}.summary.totalSteps;
        bestPriority = feasible{1}.priority;

        for i = 2:numel(feasible)
            s = feasible{i}.summary.totalSteps;
            p = feasible{i}.priority;
            if (s < bestSteps) || (s == bestSteps && p < bestPriority)
                bestIdx = i;
                bestSteps = s;
                bestPriority = p;
            end
        end

        bestPath = feasible{bestIdx}.path;
        bestRouteType = feasible{bestIdx}.name;
        bestSummary = feasible{bestIdx}.summary;
        return;
    end

    searchPads = [0.003, 0.006, 0.010, 0.015, 0.020];

    for i = 1:numel(searchPads)
        [okAstar, pathAstar] = aStarAxisSearch( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, searchPads(i), plannerOptions);

        if okAstar
            bestPath = pathAstar;
            bestRouteType = sprintf('astar_axis_pad_%0.1fmm', searchPads(i)*1e3);
            bestSummary = summarizeAxisPath(pathAstar);
            return;
        end
    end

    error(['当前目标点无法找到无碰撞路径。', ...
           '说明：axis_only 规则路径失败，且当前 A* 搜索范围内也未找到可行路径。', ...
           '建议继续扩大 searchPads，或调整放置顺序 placementOrderMode。']);
end

function wps = buildDirectWaypointsXZY(startPt, targetPt)
    wps = [
        targetPt(1), startPt(2), startPt(3);
        targetPt(1), startPt(2), targetPt(3);
        targetPt(1), targetPt(2), targetPt(3);
    ];
    wps = removeDuplicateWaypoints(startPt, wps);
end

function wps2 = removeDuplicateWaypoints(startPt, wps)
    wps2 = [];
    prev = startPt;

    for i = 1:size(wps,1)
        if norm(wps(i,:) - prev) > 1e-12
            wps2 = [wps2; wps(i,:)]; %#ok<AGROW>
            prev = wps(i,:);
        end
    end
end

function [ok, path, summary] = buildRouteAndCheck(startPt, waypoints, obstacles, clearance, allowedSteps, plannerOptions, targetPt)
    curr = startPt;
    path = curr;
    ok = true;
    summary = initAxisSummary();
    lastAxis = '';

    for i = 1:size(waypoints,1)
        wp = waypoints(i,:);
        delta = wp - curr;
        nz = find(abs(delta) > 1e-12);

        if isempty(nz)
            continue;
        end

        if numel(nz) ~= 1
            ok = false;
            path = [];
            summary = [];
            return;
        end

        dim = nz(1);
        ax = dim2axis(dim);

        if ~isempty(lastAxis) && ~strcmp(lastAxis, ax)
            summary.turnCount = summary.turnCount + 1;
        end

        lastAxis = ax;
        summary.routeAxes{end+1} = ax; %#ok<AGROW>

        stepList = decomposeDelta(delta(dim), allowedSteps);

        for k = 1:numel(stepList)
            next = curr;
            next(dim) = curr(dim) + stepList(k);

            if segmentCollidesWithAnySphere(curr, next, obstacles, clearance) || ~segmentIsValidWithRod(curr, next, plannerOptions, targetPt)
                ok = false;
                path = [];
                summary = [];
                return;
            end

            path = [path; next]; %#ok<AGROW>
            curr = next;
            summary = updateAxisStepSummary(summary, abs(stepList(k)));
        end
    end
end

function stepList = decomposeDelta(delta, allowedSteps)
    remain = abs(delta);
    sgn = sign(delta);
    stepList = [];

    if remain < 1e-12
        return;
    end

    for i = 1:numel(allowedSteps)
        st = allowedSteps(i);
        count = floor(remain / st + 1e-10);

        if count > 0
            stepList = [stepList, sgn * st * ones(1, count)]; %#ok<AGROW>
            remain = remain - count * st;
        end
    end

    if remain > 1e-10
        error('位移 %.9f m 无法由 0.5/0.2/0.1 mm 精确分解，请检查目标点是否在 0.1 mm 网格上。', delta);
    end
end

function summary = initAxisSummary()
    summary.motionMode = 'axis_only';
    summary.routeAxes = {};
    summary.step_0p5_mm = 0;
    summary.step_0p2_mm = 0;
    summary.step_0p1_mm = 0;
    summary.totalSteps  = 0;
    summary.turnCount   = 0;
    summary.totalLength_m = 0;
end

function summary = updateAxisStepSummary(summary, st)
    if abs(st - 0.0005) < 1e-12
        summary.step_0p5_mm = summary.step_0p5_mm + 1;
    elseif abs(st - 0.0002) < 1e-12
        summary.step_0p2_mm = summary.step_0p2_mm + 1;
    elseif abs(st - 0.0001) < 1e-12
        summary.step_0p1_mm = summary.step_0p1_mm + 1;
    else
        error('出现未定义单轴步长 %.9f m。', st);
    end

    summary.totalSteps = summary.totalSteps + 1;
    summary.totalLength_m = summary.totalLength_m + st;
end

function tf = segmentCollidesWithAnySphere(p0, p1, centers, clearance)
    tf = false;

    if isempty(centers)
        return;
    end

    for i = 1:size(centers,1)
        d = pointToSegmentDistance(centers(i,:), p0, p1);
        if d < clearance - 1e-12
            tf = true;
            return;
        end
    end
end

function d = pointToSegmentDistance(c, a, b)
    ab = b - a;
    denom = dot(ab, ab);

    if denom < 1e-24
        d = norm(c - a);
        return;
    end

    t = dot(c - a, ab) / denom;
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(c - proj);
end

function ax = dim2axis(dim)
    switch dim
        case 1
            ax = 'X';
        case 2
            ax = 'Y';
        case 3
            ax = 'Z';
        otherwise
            error('非法维度');
    end
end

function val2 = snapValue(val, originVal, gridBase)
    val2 = originVal + round((val - originVal) / gridBase) * gridBase;
end

function [ok, path] = aStarAxisSearch( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, pad, plannerOptions)

    gridOrigin = startPt;
    startIdx = pointToGridIndex(startPt, gridBase, gridOrigin);
    targetIdx = pointToGridIndex(targetPt, gridBase, gridOrigin);
    obsAll = [obstacles; startPt; targetPt];
    mins = floor((min(obsAll, [], 1) - pad - gridOrigin) / gridBase);
    maxs = ceil((max(obsAll, [], 1) + pad - gridOrigin) / gridBase);
    directions = [
         1  0  0;
        -1  0  0;
         0  1  0;
         0 -1  0;
         0  0  1;
         0  0 -1];

    [ok, path] = runAStarSearch(startIdx, targetIdx, mins, maxs, directions, ...
        obstacles, clearance, allowedSteps, gridBase, gridOrigin, 'axis', plannerOptions, targetPt);
end

function [ok, path] = aStarMultiAxisSearch( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, pad, plannerOptions)

    gridOrigin = startPt;
    startIdx = pointToGridIndex(startPt, gridBase, gridOrigin);
    targetIdx = pointToGridIndex(targetPt, gridBase, gridOrigin);
    obsAll = [obstacles; startPt; targetPt];
    mins = floor((min(obsAll, [], 1) - pad - gridOrigin) / gridBase);
    maxs = ceil((max(obsAll, [], 1) + pad - gridOrigin) / gridBase);
    directions = generate26Directions();

    [ok, path] = runAStarSearch(startIdx, targetIdx, mins, maxs, directions, ...
        obstacles, clearance, allowedSteps, gridBase, gridOrigin, 'multi', plannerOptions, targetPt);
end

function directions = generate26Directions()
    directions = [];
    for ix = -1:1
        for iy = -1:1
            for iz = -1:1
                v = [ix, iy, iz];
                if any(v ~= 0)
                    directions = [directions; v]; %#ok<AGROW>
                end
            end
        end
    end
end

function [ok, path] = runAStarSearch(startIdx, targetIdx, mins, maxs, directions, ...
    obstacles, clearance, allowedSteps, gridBase, gridOrigin, heuristicMode, plannerOptions, targetPt)

    ok = false;
    path = [];
    startKey = idxKey(startIdx);
    targetKey = idxKey(targetIdx);
    openKeys = {startKey};
    openF = heuristicGrid(startIdx, targetIdx, allowedSteps, gridBase, heuristicMode);
    gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
    gScore(startKey) = 0;

    cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'char');
    nodeStore = containers.Map('KeyType', 'char', 'ValueType', 'any');
    nodeStore(startKey) = startIdx;

    closed = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    maxExpand = 150000;
    expandCount = 0;

    while ~isempty(openKeys)
        expandCount = expandCount + 1;
        if expandCount > maxExpand
            return;
        end

        [~, bestPos] = min(openF);
        currentKey = openKeys{bestPos};
        currentIdx = nodeStore(currentKey);

        openKeys(bestPos) = [];
        openF(bestPos) = [];

        if strcmp(currentKey, targetKey)
            path = reconstructPath(cameFrom, nodeStore, currentKey, gridBase, gridOrigin);
            ok = true;
            return;
        end

        closed(currentKey) = true;
        nbrs = generateNeighborsByDirections(currentIdx, mins, maxs, directions, allowedSteps, gridBase);

        for i = 1:size(nbrs,1)
            nbrIdx = nbrs(i,:);
            nbrKey = idxKey(nbrIdx);

            if isKey(closed, nbrKey)
                continue;
            end

            p0 = gridIndexToPoint(currentIdx, gridBase, gridOrigin);
            p1 = gridIndexToPoint(nbrIdx, gridBase, gridOrigin);

            if ~isPathCollisionFree([p0; p1], obstacles, clearance, plannerOptions, targetPt)
                continue;
            end

            tentativeG = gScore(currentKey) + 1;

            if ~isKey(gScore, nbrKey) || tentativeG < gScore(nbrKey)
                cameFrom(nbrKey) = currentKey;
                gScore(nbrKey) = tentativeG;
                nodeStore(nbrKey) = nbrIdx;

                f = tentativeG + heuristicGrid(nbrIdx, targetIdx, allowedSteps, gridBase, heuristicMode);
                pos = find(strcmp(openKeys, nbrKey), 1);
                if isempty(pos)
                    openKeys{end+1} = nbrKey; %#ok<AGROW>
                    openF(end+1) = f; %#ok<AGROW>
                else
                    openF(pos) = f;
                end
            end
        end
    end
end

function nbrs = generateNeighborsByDirections(currIdx, mins, maxs, directions, allowedSteps, gridBase)
    stepUnits = round(allowedSteps / gridBase);
    nbrs = [];

    for i = 1:size(directions,1)
        dirVec = directions(i,:);
        for j = 1:numel(stepUnits)
            nxt = currIdx + dirVec * stepUnits(j);
            if all(nxt >= mins) && all(nxt <= maxs)
                nbrs = [nbrs; nxt]; %#ok<AGROW>
            end
        end
    end
end

function h = heuristicGrid(idx, targetIdx, allowedSteps, gridBase, mode)
    maxStepUnit = max(round(allowedSteps / gridBase));

    if strcmpi(mode, 'multi')
        h = ceil(max(abs(idx - targetIdx)) / maxStepUnit);
    else
        manhattan = sum(abs(idx - targetIdx));
        h = ceil(manhattan / maxStepUnit);
    end
end

function path = reconstructPath(cameFrom, nodeStore, currentKey, gridBase, gridOrigin)
    keyList = {currentKey};

    while isKey(cameFrom, currentKey)
        currentKey = cameFrom(currentKey);
        keyList = [{currentKey}, keyList]; %#ok<AGROW>
    end

    path = zeros(numel(keyList), 3);
    for i = 1:numel(keyList)
        idx = nodeStore(keyList{i});
        path(i,:) = gridIndexToPoint(idx, gridBase, gridOrigin);
    end
end

function idx = pointToGridIndex(pt, gridBase, gridOrigin)
    idx = round((pt - gridOrigin) / gridBase);
end

function pt = gridIndexToPoint(idx, gridBase, gridOrigin)
    pt = gridOrigin + idx * gridBase;
end

function key = idxKey(idx)
    key = sprintf('%d_%d_%d', idx(1), idx(2), idx(3));
end

function summary = summarizeAxisPath(path)
    summary = initAxisSummary();

    if size(path,1) <= 1
        return;
    end

    lastAxis = '';

    for i = 2:size(path,1)
        dp = path(i,:) - path(i-1,:);
        dim = find(abs(dp) > 1e-12);

        if numel(dim) ~= 1
            continue;
        end

        ax = dim2axis(dim);

        if ~isempty(lastAxis) && ~strcmp(lastAxis, ax)
            summary.turnCount = summary.turnCount + 1;
        end

        lastAxis = ax;
        summary.routeAxes{end+1} = ax; %#ok<AGROW>
        summary = updateAxisStepSummary(summary, abs(dp(dim)));
    end
end

function startConstraint = finalizeStartHorizontalConstraint(startConstraint)
    startConstraint.exitPoint = startConstraint.startPoint;
    startConstraint.exitPoint(startConstraint.forwardAxis) = ...
        startConstraint.startPoint(startConstraint.forwardAxis) + ...
        startConstraint.forwardDirection * startConstraint.forwardDistance;
end

function tf = isStartHorizontalConstraintEnabled(plannerOptions)
    tf = isfield(plannerOptions, 'enableStartHorizontalConstraint') && ...
         plannerOptions.enableStartHorizontalConstraint && ...
         isfield(plannerOptions, 'startConstraint') && ...
         ~isempty(plannerOptions.startConstraint);
end

function tf = isStartHorizontalConstraintActive(startPt, plannerOptions)
    tf = false;
    if ~isStartHorizontalConstraintEnabled(plannerOptions)
        return;
    end
    sc = plannerOptions.startConstraint;
    tf = norm(startPt - sc.startPoint) < 1e-12;
end

function plannerOptions2 = disableStartHorizontalConstraint(plannerOptions)
    plannerOptions2 = plannerOptions;
    plannerOptions2.enableStartHorizontalConstraint = false;
end

function [ok, prefixPath, exitPt, failReason] = buildStartHorizontalPrefixPath( ...
    startPt, obstacles, clearance, plannerOptions, targetPt)

    ok = false;
    prefixPath = [];
    failReason = '';
    sc = plannerOptions.startConstraint;
    exitPt = sc.exitPoint;

    if abs(exitPt(sc.heightAxis) - startPt(sc.heightAxis)) > sc.verticalTolerance
        failReason = 'exitPoint 与 startPoint 的高度不一致';
        return;
    end

    stepLen = sc.pathStep;
    if ~isfinite(stepLen) || stepLen <= 0
        if isfield(plannerOptions, 'multiAxisPathStep') && plannerOptions.multiAxisPathStep > 0
            stepLen = plannerOptions.multiAxisPathStep;
        else
            stepLen = 0.1e-3;
        end
    end

    prefixPath = buildLinePath(startPt, exitPt, stepLen);

    for ii = 2:size(prefixPath, 1)
        if abs(prefixPath(ii, sc.heightAxis) - prefixPath(ii-1, sc.heightAxis)) > sc.verticalTolerance
            failReason = '起始水平前进段发生了高度变化';
            prefixPath = [];
            return;
        end
    end

    if ~isPathCollisionFree(prefixPath, obstacles, clearance, plannerOptions, targetPt)
        failReason = '起始水平前进段与已放置小球或长杆约束冲突';
        prefixPath = [];
        return;
    end

    ok = true;
end

function info = fillStartHorizontalInfo(info, plannerOptions)
    if isStartHorizontalConstraintEnabled(plannerOptions)
        info.startHorizontalConstraintEnabled = true;
        info.startHorizontalDistance = plannerOptions.startConstraint.forwardDistance;
        info.startHorizontalExitPoint = plannerOptions.startConstraint.exitPoint;
    end
end

function info = updateStartHorizontalInfoFromRoute(info, routeType, plannerOptions)
    if isStartHorizontalConstraintEnabled(plannerOptions)
        info.startHorizontalConstraintEnabled = true;
        info.startHorizontalDistance = plannerOptions.startConstraint.forwardDistance;
        info.startHorizontalExitPoint = plannerOptions.startConstraint.exitPoint;
        if contains(lower(routeType), 'start_horizontal')
            info.startHorizontalUsed = true;
            info.startHorizontalRouteType = 'start_horizontal_constant_y_forward_4mm';
        end
    end
end

function supportObjects = buildSupportObjectsFromTargets(targets, supportConfig, sphereR, rod)
    supportObjectCells = {};
    objIdx = 0;
    targetIDs = supportConfig.targetGlobalIDs(:)';
    rodTopY = rod.center(rod.heightAxis) + rod.size(rod.heightAxis)/2;

    for id = targetIDs
        idx = find([targets.globalID] == id, 1, 'first');
        if isempty(idx)
            warning('未找到 supportConfig.targetGlobalIDs 中的 globalID = %g，跳过该支撑物。', id);
            continue;
        end

        ballC = targets(idx).targetPoint;
        saddleTopY = ballC(2) - sphereR - supportConfig.saddleTopGap;
        saddleCenterY = saddleTopY - supportConfig.saddleSize(2)/2;
        objIdx = objIdx + 1;
        supportObjectCells{objIdx} = makeSupportBox( ...
            sprintf('support_ID%d_saddle_base', id), 'saddle_base', ...
            [ballC(1), saddleCenterY, ballC(3)], supportConfig.saddleSize, ...
            id, ballC, supportConfig, sphereR, [0.10 0.45 0.85], supportConfig.visualAlpha); %#ok<AGROW>

        armTopY = saddleCenterY - supportConfig.saddleSize(2)/2 - supportConfig.armClearGapToSaddle;
        armCenterY = armTopY - supportConfig.armThicknessY/2;

        x0 = rod.center(1);
        x1 = ballC(1);
        dirSign = sign(x1 - x0);
        if abs(dirSign) < 1e-12
            dirSign = 1;
        end

        rodEdgeX = x0 + dirSign * rod.size(1)/2;
        saddleEdgeX = ballC(1) - dirSign * supportConfig.saddleSize(1)/2;
        armLengthX = abs(saddleEdgeX - rodEdgeX);

        if armLengthX > 0.2e-3
            armCenterX = (saddleEdgeX + rodEdgeX) / 2;
            armCenter = [armCenterX, armCenterY, ballC(3)];
            armSize = [armLengthX, supportConfig.armThicknessY, supportConfig.armWidthZ];
            objIdx = objIdx + 1;
            supportObjectCells{objIdx} = makeSupportBox( ...
                sprintf('support_ID%d_arm', id), 'support_arm', ...
                armCenter, armSize, ...
                id, ballC, supportConfig, sphereR, [0.15 0.60 0.95], supportConfig.visualAlpha); %#ok<AGROW>
        end

        armBottomY = armCenterY - supportConfig.armThicknessY/2;
        riserHeight = max(armBottomY - rodTopY, 0.2e-3);
        riserCenterY = rodTopY + riserHeight/2;
        riserCenter = [rod.center(1), riserCenterY, ballC(3)];
        riserSize = [supportConfig.riserSizeXZ(1), riserHeight, supportConfig.riserSizeXZ(2)];
        objIdx = objIdx + 1;
        supportObjectCells{objIdx} = makeSupportBox( ...
            sprintf('support_ID%d_riser', id), 'support_riser', ...
            riserCenter, riserSize, ...
            id, ballC, supportConfig, sphereR, [0.15 0.55 0.85], supportConfig.visualAlpha); %#ok<AGROW>
    end

    if isempty(supportObjectCells)
        supportObjects = struct([]);
    else
        supportObjects = [supportObjectCells{:}];
    end
end

function obj = makeSupportBox(name, typeName, center, sizeVec, allowID, allowPt, cfg, sphereR, color, alphaVal)
    obj.name = name;
    obj.type = typeName;
    obj.center = center;
    obj.size = sizeVec;
    obj.halfSize = sizeVec / 2;
    obj.minCorner = center - obj.halfSize;
    obj.maxCorner = center + obj.halfSize;
    obj.allowContactGlobalIDs = allowID;
    obj.allowContactTargetPoint = allowPt;
    obj.supportMargin = cfg.supportMargin;
    obj.contactAllowance = cfg.contactAllowance;
    obj.endpointRelaxRadius = cfg.endpointRelaxRadius;
    obj.color = color;
    obj.visualAlpha = alphaVal;
    inflateStrict = sphereR + cfg.supportMargin;
    inflateRelaxed = max(0, sphereR - cfg.contactAllowance + cfg.supportMargin);
    obj.collisionInflate = inflateStrict;
    obj.relaxedCollisionInflate = inflateRelaxed;
    obj.inflatedMinCorner = obj.minCorner - inflateStrict;
    obj.inflatedMaxCorner = obj.maxCorner + inflateStrict;
    obj.relaxedInflatedMinCorner = obj.minCorner - inflateRelaxed;
    obj.relaxedInflatedMaxCorner = obj.maxCorner + inflateRelaxed;
end

function tf = isSupportObjectsEnabled(plannerOptions)
    tf = isfield(plannerOptions, 'enableSupportObjects') && plannerOptions.enableSupportObjects && ...
         isfield(plannerOptions, 'supportObjects') && ~isempty(plannerOptions.supportObjects);
end

function info = fillSupportInfoForTarget(info, targetPt, plannerOptions)
    if ~isSupportObjectsEnabled(plannerOptions)
        return;
    end

    info.supportObjectsEnabled = true;
    info.supportCollisionChecked = true;

    [allowed, names] = supportContactAllowedForTarget(targetPt, plannerOptions.supportObjects);
    info.supportContactAllowed = allowed;
    info.supportContactObjectNames = names;
end

function [allowed, names] = supportContactAllowedForTarget(targetPt, supportObjects)
    allowed = false;
    names = {};
    if isempty(supportObjects) || any(~isfinite(targetPt))
        return;
    end
    for kk = 1:numel(supportObjects)
        obj = supportObjects(kk);
        if isfield(obj, 'allowContactTargetPoint') && norm(targetPt - obj.allowContactTargetPoint) < 1e-9
            allowed = true;
            names{end+1} = obj.name; %#ok<AGROW>
        end
    end
end

function valid = segmentIsValidWithSupportObjects(p0, p1, plannerOptions, targetPt)
    valid = true;

    if ~isSupportObjectsEnabled(plannerOptions)
        return;
    end

    supportObjects = plannerOptions.supportObjects;

    for kk = 1:numel(supportObjects)
        obj = supportObjects(kk);
        useRelaxed = false;

        if nargin >= 4 && all(isfinite(targetPt)) && isfield(obj, 'allowContactTargetPoint')
            isAllowedTarget = norm(targetPt - obj.allowContactTargetPoint) < 1e-9;
            nearEndpoint = min(norm(p0 - targetPt), norm(p1 - targetPt)) <= obj.endpointRelaxRadius + 1e-12;
            if isAllowedTarget && nearEndpoint
                useRelaxed = true;
            end
        end

        if useRelaxed
            boxMin = obj.relaxedInflatedMinCorner;
            boxMax = obj.relaxedInflatedMaxCorner;
        else
            boxMin = obj.inflatedMinCorner;
            boxMax = obj.inflatedMaxCorner;
        end

        if segmentIntersectsAABB(p0, p1, boxMin, boxMax)
            valid = false;
            return;
        end
    end
end

function rod = finalizeRodOptions(rod, sphereR)
    rod.halfSize = rod.size / 2;
    rod.minCorner = rod.center - rod.halfSize;
    rod.maxCorner = rod.center + rod.halfSize;
    rod.topY = rod.center(rod.heightAxis) + rod.size(rod.heightAxis) / 2;
    rod.safeY = rod.topY + sphereR + rod.extraHeightGap;

    inflate = sphereR + rod.collisionMargin;
    rod.collisionInflate = inflate;
    rod.inflatedMinCorner = rod.minCorner - inflate;
    rod.inflatedMaxCorner = rod.maxCorner + inflate;
    rod.aboveMinX = rod.center(1) - rod.size(1)/2 - rod.aboveBufferX;
    rod.aboveMaxX = rod.center(1) + rod.size(1)/2 + rod.aboveBufferX;
    rod.aboveMinZ = rod.center(3) - rod.size(3)/2 - rod.aboveBufferZ;
    rod.aboveMaxZ = rod.center(3) + rod.size(3)/2 + rod.aboveBufferZ;
end

function tf = isRodObstacleEnabled(plannerOptions)
    tf = isfield(plannerOptions, 'enableRodObstacle') && plannerOptions.enableRodObstacle && ...
         isfield(plannerOptions, 'rod') && ~isempty(plannerOptions.rod);
end

function info = fillRodInfoForTarget(info, targetPt, plannerOptions)
    if isRodObstacleEnabled(plannerOptions)
        info.rodObstacleEnabled = true;
        info.rodSafeY = plannerOptions.rod.safeY;
        info.targetAboveRodRegion = isTargetAboveRodRegion(targetPt, plannerOptions.rod);
        info.rodConstraintChecked = true;
    end
end

function tf = isTargetAboveRodRegion(pt, rod)
    tf = pt(1) >= rod.aboveMinX - 1e-12 && pt(1) <= rod.aboveMaxX + 1e-12 && ...
         pt(3) >= rod.aboveMinZ - 1e-12 && pt(3) <= rod.aboveMaxZ + 1e-12 && ...
         pt(rod.heightAxis) >= rod.topY - 1e-12;
end

function valid = segmentIsValidWithRod(p0, p1, plannerOptions, targetPt)
    valid = true;

    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end

    rod = plannerOptions.rod;

    if segmentIntersectsAABB(p0, p1, rod.inflatedMinCorner, rod.inflatedMaxCorner)
        valid = false;
        return;
    end

    if segmentTouchesRodAboveRegion(p0, p1, rod)
        dy = abs(p1(rod.heightAxis) - p0(rod.heightAxis));
        if dy > rod.verticalTolerance
            valid = false;
            return;
        end

        targetIsAboveRod = false;
        if nargin >= 4 && all(isfinite(targetPt))
            targetIsAboveRod = isTargetAboveRodRegion(targetPt, rod);
        end

        allowLowerTarget = targetIsAboveRod && isfield(rod, 'targetAboveRodMode') && ...
            strcmpi(rod.targetAboveRodMode, 'allow_lower_target');

        if ~allowLowerTarget
            ySeg = min(p0(rod.heightAxis), p1(rod.heightAxis));
            if ySeg < rod.safeY - 1e-12
                valid = false;
                return;
            end
        end
    end
end

function tf = segmentTouchesRodAboveRegion(p0, p1, rod)
    segMinX = min(p0(1), p1(1));
    segMaxX = max(p0(1), p1(1));
    segMinZ = min(p0(3), p1(3));
    segMaxZ = max(p0(3), p1(3));
    segMaxY = max(p0(rod.heightAxis), p1(rod.heightAxis));
    overlapX = segMaxX >= rod.aboveMinX - 1e-12 && segMinX <= rod.aboveMaxX + 1e-12;
    overlapZ = segMaxZ >= rod.aboveMinZ - 1e-12 && segMinZ <= rod.aboveMaxZ + 1e-12;
    aboveTop = segMaxY >= rod.topY - 1e-12;
    tf = overlapX && overlapZ && aboveTop;
end

function tf = segmentIntersectsAABB(p0, p1, boxMin, boxMax)
    tmin = 0;
    tmax = 1;
    d = p1 - p0;

    for ax = 1:3
        if abs(d(ax)) < 1e-15
            if p0(ax) < boxMin(ax) || p0(ax) > boxMax(ax)
                tf = false;
                return;
            end
        else
            invD = 1 / d(ax);
            t1 = (boxMin(ax) - p0(ax)) * invD;
            t2 = (boxMax(ax) - p0(ax)) * invD;
            if t1 > t2
                tmp = t1; t1 = t2; t2 = tmp;
            end
            tmin = max(tmin, t1);
            tmax = min(tmax, t2);
            if tmin > tmax
                tf = false;
                return;
            end
        end
    end

    tf = true;
end

function tf = isTargetAboveRodForHorizontalEntry(targetPt, plannerOptions)
    tf = false;
    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end
    rod = plannerOptions.rod;
    tf = isTargetAboveRodRegion(targetPt, rod);
end

function wpsSet = buildRodAboveTargetHorizontalEntryWaypointCandidates(startPt, targetPt, rod)
    wpsSet = {};

    if ~isTargetAboveRodRegion(targetPt, rod)
        return;
    end

    yTarget = targetPt(rod.heightAxis);
    gap = max(0.5e-3, rod.aboveBufferX);
    xLeftOutside  = rod.aboveMinX - gap;
    xRightOutside = rod.aboveMaxX + gap;
    v = [targetPt(1) - rod.center(1), targetPt(3) - rod.center(3)];
    if norm(v) < 1e-12
        v = [targetPt(1) - startPt(1), targetPt(3) - startPt(3)];
    end
    if norm(v) < 1e-12
        v = [1, 0];
    end
    u0 = v / norm(v);
    entryDistances = [3.0e-3, 4.0e-3, 5.0e-3, 6.0e-3, 8.0e-3];
    angleOffsetsDeg = [0, -15, 15, -30, 30];
    radialCandidates = {};
    for id = 1:numel(entryDistances)
        for ia = 1:numel(angleOffsetsDeg)
            ang = deg2rad_local(angleOffsetsDeg(ia));
            R = [cos(ang), -sin(ang); sin(ang), cos(ang)];
            u = (R * u0(:)).';
            if norm(u) < 1e-12
                continue;
            end
            u = u / norm(u);
            preX = targetPt(1) + entryDistances(id) * u(1);
            preZ = targetPt(3) + entryDistances(id) * u(2);
            wps = [
                startPt(1), yTarget,   startPt(3);
                startPt(1), yTarget,   preZ;
                preX,       yTarget,   preZ;
                targetPt(1), targetPt(2), targetPt(3)
            ];
            radialCandidates{end+1} = wps; %#ok<AGROW>
        end
    end

    leftWps = [
        xLeftOutside,  startPt(2), startPt(3);
        xLeftOutside,  yTarget,   startPt(3);
        xLeftOutside,  yTarget,   targetPt(3);
        targetPt(1),   targetPt(2), targetPt(3)
    ];

    rightWps = [
        xRightOutside, startPt(2), startPt(3);
        xRightOutside, yTarget,   startPt(3);
        xRightOutside, yTarget,   targetPt(3);
        targetPt(1),   targetPt(2), targetPt(3)
    ];

    if startPt(1) < rod.aboveMinX
        sideCandidates = {leftWps, rightWps};
    elseif startPt(1) > rod.aboveMaxX
        sideCandidates = {rightWps, leftWps};
    else
        if abs(startPt(1) - xLeftOutside) <= abs(startPt(1) - xRightOutside)
            sideCandidates = {leftWps, rightWps};
        else
            sideCandidates = {rightWps, leftWps};
        end
    end

    wpsSet = [radialCandidates, sideCandidates];
end

function [ok, bestPath, bestRouteType, bestSummary] = planRodAboveTarget2DXZPath( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end

    rod = plannerOptions.rod;
    if ~isTargetAboveRodRegion(targetPt, rod)
        return;
    end

    stepLen = plannerOptions.multiAxisPathStep;
    entryPts = generateRodAbove2DEntryPoints(startPt, targetPt, rod, gridBase);
    if isempty(entryPts)
        return;
    end

    searchPads = [0.0025, 0.0040, 0.0060, 0.0080, 0.0120, 0.0160];
    bestSteps = inf;
    bestLen = inf;
    bestEntryIdx = inf;

    for e = 1:size(entryPts,1)
        entryPt = entryPts(e,:);
        entryRouteCandidates = buildEntryRouteCandidatesForRodAbove2D(startPt, entryPt, targetPt, rod);

        for r = 1:numel(entryRouteCandidates)
            wpsToEntry = removeDuplicateWaypoints(startPt, entryRouteCandidates{r});
            [okEntry, entryPath, ~] = buildMultiAxisRouteAndCheck( ...
                startPt, wpsToEntry, obstacles, clearance, stepLen, plannerOptions, targetPt);

            if ~okEntry
                continue;
            end

            directPlanar = buildLinePath(entryPt, targetPt, stepLen);
            if isPathCollisionFree(directPlanar, obstacles, clearance, plannerOptions, targetPt)
                fullPath = concatPaths(entryPath, directPlanar);
                summary = summarizeMultiAxisPath(fullPath);
                if summary.totalSteps < bestSteps || (summary.totalSteps == bestSteps && summary.totalLength_m < bestLen)
                    ok = true;
                    bestPath = fullPath;
                    bestRouteType = sprintf('rod_above_target_2d_xz_direct_entry_%02d', e);
                    bestSummary = summary;
                    bestSteps = summary.totalSteps;
                    bestLen = summary.totalLength_m;
                    bestEntryIdx = e; %#ok<NASGU>
                end
                continue;
            end

            for pidx = 1:numel(searchPads)
                [ok2D, planarPath] = aStarRodAbove2DXZSearch( ...
                    entryPt, targetPt, obstacles, clearance, allowedSteps, gridBase, searchPads(pidx), plannerOptions);

                if ~ok2D
                    continue;
                end

                fullPath = concatPaths(entryPath, planarPath);
                if ~isPathCollisionFree(fullPath, obstacles, clearance, plannerOptions, targetPt)
                    continue;
                end

                summary = summarizeMultiAxisPath(fullPath);
                if summary.totalSteps < bestSteps || (summary.totalSteps == bestSteps && summary.totalLength_m < bestLen)
                    ok = true;
                    bestPath = fullPath;
                    bestRouteType = sprintf('rod_above_target_2d_xz_astar_entry_%02d_pad_%0.1fmm', e, searchPads(pidx)*1e3);
                    bestSummary = summary;
                    bestSteps = summary.totalSteps;
                    bestLen = summary.totalLength_m;
                    bestEntryIdx = e; %#ok<NASGU>
                end

                break;
            end
        end

        if ok && bestSteps < 250
            return;
        end
    end
end

function entryPts = generateRodAbove2DEntryPoints(startPt, targetPt, rod, gridBase)
    entryPts = [];
    yTarget = targetPt(rod.heightAxis);
    v = [targetPt(1) - rod.center(1), targetPt(3) - rod.center(3)];
    if norm(v) < 1e-12
        v = [targetPt(1) - startPt(1), targetPt(3) - startPt(3)];
    end
    if norm(v) < 1e-12
        v = [1, 0];
    end
    u0 = v / norm(v);
    entryDistances = [2.5e-3, 3.5e-3, 5.0e-3, 6.5e-3, 8.0e-3, 10.0e-3, 12.0e-3];
    angleOffsetsDeg = [0, -15, 15, -30, 30, -45, 45, -60, 60, -90, 90, 120, -120, 180];

    for id = 1:numel(entryDistances)
        for ia = 1:numel(angleOffsetsDeg)
            ang = deg2rad_local(angleOffsetsDeg(ia));
            R = [cos(ang), -sin(ang); sin(ang), cos(ang)];
            u = (R * u0(:)).';
            if norm(u) < 1e-12
                continue;
            end
            u = u / norm(u);
            x = targetPt(1) + entryDistances(id) * u(1);
            z = targetPt(3) + entryDistances(id) * u(2);
            x = snapValue(x, targetPt(1), gridBase);
            z = snapValue(z, targetPt(3), gridBase);
            entryPts = [entryPts; x, yTarget, z]; %#ok<AGROW>
        end
    end

    gap = max(0.5e-3, rod.aboveBufferX);
    xSide = [rod.aboveMinX - gap, rod.aboveMaxX + gap];
    zOffsets = [-12e-3, -8e-3, -5e-3, -3e-3, 0, 3e-3, 5e-3, 8e-3, 12e-3];
    for xs = 1:numel(xSide)
        for iz = 1:numel(zOffsets)
            x = snapValue(xSide(xs), targetPt(1), gridBase);
            z = snapValue(targetPt(3) + zOffsets(iz), targetPt(3), gridBase);
            entryPts = [entryPts; x, yTarget, z]; %#ok<AGROW>
        end
    end

    if ~isempty(entryPts)
        key = round(entryPts / gridBase);
        [~, ia, ~] = unique(key, 'rows', 'stable');
        entryPts = entryPts(ia,:);
        d = sqrt(sum((entryPts - targetPt).^2, 2));
        [~, ord] = sort(d, 'ascend');
        entryPts = entryPts(ord,:);
    end
end

function routes = buildEntryRouteCandidatesForRodAbove2D(startPt, entryPt, targetPt, rod)
    %#ok<INUSD>
    yTarget = targetPt(rod.heightAxis);
    routes = {};

    routes{end+1} = [
        startPt(1), yTarget,   startPt(3);
        entryPt(1), entryPt(2), entryPt(3)
    ];

    routes{end+1} = [
        startPt(1), yTarget,   startPt(3);
        startPt(1), yTarget,   entryPt(3);
        entryPt(1), entryPt(2), entryPt(3)
    ];

    routes{end+1} = [
        startPt(1), yTarget,   startPt(3);
        entryPt(1), yTarget,   startPt(3);
        entryPt(1), entryPt(2), entryPt(3)
    ];
end

function [ok, path] = aStarRodAbove2DXZSearch( ...
    entryPt, targetPt, obstacles, clearance, allowedSteps, gridBase, pad, plannerOptions)

    ok = false;
    path = [];
    yPlane = targetPt(plannerOptions.rod.heightAxis);
    gridOriginXZ = [targetPt(1), targetPt(3)];
    startIdx = pointToGridIndex2D(entryPt([1,3]), gridBase, gridOriginXZ);
    targetIdx = [0, 0];
    ptsXZ = [entryPt([1,3]); targetPt([1,3])];
    mins = floor((min(ptsXZ, [], 1) - pad - gridOriginXZ) / gridBase);
    maxs = ceil((max(ptsXZ, [], 1) + pad - gridOriginXZ) / gridBase);
    directions = [
         1  0;
        -1  0;
         0  1;
         0 -1;
         1  1;
         1 -1;
        -1  1;
        -1 -1];

    [ok, path] = runAStar2DXZSearch(startIdx, targetIdx, mins, maxs, directions, ...
        obstacles, clearance, allowedSteps, gridBase, gridOriginXZ, yPlane, plannerOptions, targetPt);
end

function [ok, path] = runAStar2DXZSearch(startIdx, targetIdx, mins, maxs, directions, ...
    obstacles, clearance, allowedSteps, gridBase, gridOriginXZ, yPlane, plannerOptions, targetPt)

    ok = false;
    path = [];
    startKey = idxKey2D(startIdx);
    targetKey = idxKey2D(targetIdx);
    openKeys = {startKey};
    openF = heuristicGrid2D(startIdx, targetIdx, allowedSteps, gridBase);
    gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
    gScore(startKey) = 0;

    cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'char');
    nodeStore = containers.Map('KeyType', 'char', 'ValueType', 'any');
    nodeStore(startKey) = startIdx;

    closed = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    maxExpand = 60000;
    expandCount = 0;

    while ~isempty(openKeys)
        expandCount = expandCount + 1;
        if expandCount > maxExpand
            return;
        end

        [~, bestPos] = min(openF);
        currentKey = openKeys{bestPos};
        currentIdx = nodeStore(currentKey);

        openKeys(bestPos) = [];
        openF(bestPos) = [];

        if strcmp(currentKey, targetKey)
            path = reconstructPath2DXZ(cameFrom, nodeStore, currentKey, gridBase, gridOriginXZ, yPlane);
            ok = true;
            return;
        end

        closed(currentKey) = true;
        nbrs = generateNeighbors2DByDirections(currentIdx, mins, maxs, directions, allowedSteps, gridBase);

        for i = 1:size(nbrs,1)
            nbrIdx = nbrs(i,:);
            nbrKey = idxKey2D(nbrIdx);

            if isKey(closed, nbrKey)
                continue;
            end

            p0 = gridIndex2DToPoint(currentIdx, gridBase, gridOriginXZ, yPlane);
            p1 = gridIndex2DToPoint(nbrIdx, gridBase, gridOriginXZ, yPlane);

            if ~isPathCollisionFree([p0; p1], obstacles, clearance, plannerOptions, targetPt)
                continue;
            end

            stepCost = norm(p1 - p0) / gridBase;
            tentativeG = gScore(currentKey) + stepCost;

            if ~isKey(gScore, nbrKey) || tentativeG < gScore(nbrKey)
                cameFrom(nbrKey) = currentKey;
                gScore(nbrKey) = tentativeG;
                nodeStore(nbrKey) = nbrIdx;

                f = tentativeG + heuristicGrid2D(nbrIdx, targetIdx, allowedSteps, gridBase);
                pos = find(strcmp(openKeys, nbrKey), 1);
                if isempty(pos)
                    openKeys{end+1} = nbrKey; %#ok<AGROW>
                    openF(end+1) = f; %#ok<AGROW>
                else
                    openF(pos) = f;
                end
            end
        end
    end
end

function nbrs = generateNeighbors2DByDirections(currIdx, mins, maxs, directions, allowedSteps, gridBase)
    stepUnits = round(allowedSteps / gridBase);
    nbrs = [];
    for i = 1:size(directions,1)
        dirVec = directions(i,:);
        for j = 1:numel(stepUnits)
            nxt = currIdx + dirVec * stepUnits(j);
            if all(nxt >= mins) && all(nxt <= maxs)
                nbrs = [nbrs; nxt]; %#ok<AGROW>
            end
        end
    end
end

function h = heuristicGrid2D(idx, targetIdx, allowedSteps, gridBase)
    maxStepUnit = max(round(allowedSteps / gridBase));
    h = ceil(max(abs(idx - targetIdx)) / maxStepUnit);
end

function path = reconstructPath2DXZ(cameFrom, nodeStore, currentKey, gridBase, gridOriginXZ, yPlane)
    keyList = {currentKey};
    while isKey(cameFrom, currentKey)
        currentKey = cameFrom(currentKey);
        keyList = [{currentKey}, keyList]; %#ok<AGROW>
    end
    path = zeros(numel(keyList), 3);
    for i = 1:numel(keyList)
        idx = nodeStore(keyList{i});
        path(i,:) = gridIndex2DToPoint(idx, gridBase, gridOriginXZ, yPlane);
    end
end

function idx = pointToGridIndex2D(ptXZ, gridBase, gridOriginXZ)
    idx = round((ptXZ - gridOriginXZ) / gridBase);
end

function pt = gridIndex2DToPoint(idx, gridBase, gridOriginXZ, yPlane)
    xz = gridOriginXZ + idx * gridBase;
    pt = [xz(1), yPlane, xz(2)];
end

function key = idxKey2D(idx)
    key = sprintf('%d_%d', idx(1), idx(2));
end

function a = deg2rad_local(deg)
    a = deg * pi / 180;
end

function wps = buildRodSafeHorizontalCrossWaypoints(startPt, targetPt, rod)
    wps = [];

    if (startPt(1) < rod.aboveMinX && targetPt(1) < rod.aboveMinX) || ...
       (startPt(1) > rod.aboveMaxX && targetPt(1) > rod.aboveMaxX)
        return;
    end

    gap = max(0.5e-3, rod.aboveBufferX);
    if startPt(1) <= rod.center(1)
        xBefore = rod.aboveMinX - gap;
        xAfter  = rod.aboveMaxX + gap;
    else
        xBefore = rod.aboveMaxX + gap;
        xAfter  = rod.aboveMinX - gap;
    end

    ySafe = rod.safeY;
    zCross0 = startPt(3);
    zCross1 = targetPt(3);
    wps = [
        startPt(1), ySafe, startPt(3);
        xBefore,    ySafe, zCross0;
        xAfter,     ySafe, zCross1;
        targetPt(1), ySafe, targetPt(3);
        targetPt(1), targetPt(2), targetPt(3)
    ];
end

function drawSupportObjects(supportObjects)
    for kk = 1:numel(supportObjects)
        obj = supportObjects(kk);
        if isfield(obj, 'color')
            c = obj.color;
        else
            c = [0.10 0.45 0.85];
        end
        if isfield(obj, 'visualAlpha')
            a = obj.visualAlpha;
        else
            a = 0.45;
        end
        drawBoxColor(obj.minCorner, obj.maxCorner, c, a);
    end
end

function drawBoxColor(boxMin, boxMax, faceColor, faceAlpha)
    x = [boxMin(1), boxMax(1)];
    y = [boxMin(2), boxMax(2)];
    z = [boxMin(3), boxMax(3)];
    V = [
        x(1) y(1) z(1);
        x(2) y(1) z(1);
        x(2) y(2) z(1);
        x(1) y(2) z(1);
        x(1) y(1) z(2);
        x(2) y(1) z(2);
        x(2) y(2) z(2);
        x(1) y(2) z(2)];
    F = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8];

    patch('Vertices', V, 'Faces', F, 'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
        'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 0.6);
end

function drawRodObstacle(rod)
    drawBox(rod.minCorner, rod.maxCorner, 0.28);
    % drawBox(rod.inflatedMinCorner, rod.inflatedMaxCorner, 0.07);

    % plot3([rod.aboveMinX, rod.aboveMaxX], [rod.safeY, rod.safeY], [rod.center(3), rod.center(3)], ...
    %     'k--', 'LineWidth', 1.0);
    % text(rod.aboveMaxX, rod.safeY, rod.center(3), ' rod safeY', 'FontSize', 8);
end

function drawBox(boxMin, boxMax, faceAlpha)
    x = [boxMin(1), boxMax(1)];
    y = [boxMin(2), boxMax(2)];
    z = [boxMin(3), boxMax(3)];
    V = [
        x(1) y(1) z(1);
        x(2) y(1) z(1);
        x(2) y(2) z(1);
        x(1) y(2) z(1);
        x(1) y(1) z(2);
        x(2) y(1) z(2);
        x(2) y(2) z(2);
        x(1) y(2) z(2)];
    F = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8];

    patch('Vertices', V, 'Faces', F, 'FaceAlpha', faceAlpha, ...
        'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 0.6);
end

function writeJsonFile(filePath, dataStruct)
    try
        txt = jsonencode(dataStruct, 'PrettyPrint', true);
    catch
        txt = jsonencode(dataStruct);
    end

    fid = fopen(filePath, 'w', 'n', 'UTF-8');
    if fid == -1
        error('无法创建文件: %s', filePath);
    end
    fwrite(fid, txt, 'char');
    fclose(fid);
end

function y = roundN(x, n)
    if isempty(x)
        y = x;
        return;
    end
    scale = 10^n;
    y = round(x * scale) / scale;
end

function C = structArrayToCellArray(S)
    if isempty(S)
        C = {};
        return;
    end

    C = cell(1, numel(S));
    for k = 1:numel(S)
        C{k} = S(k);
    end
end
