clc; clear;
script_dir = fileparts(mfilename('fullpath'));
% close all;
fprintf('\n=== 正在运行版本：BUTTERFLY_WING_V3，首球强制支撑 + 150mm三杆 + 骨架贴近/转折严格避杆 ===\n');

%  ascending_spiral_adaptive_transport_path_planning.m

csvFile = fullfile(script_dir, 'wing_ball_numbering.csv');
regenerateSpiralCsv = false;

if ~isfile(csvFile)
    error('找不到蝴蝶翅膀路径规划 CSV：%s。请先运行 v7 坐标生成代码。', csvFile);
end

csvCoordUnit = 'mm';
swapYZColumns = false;
modelOffset_m = [0.0000, 0.0840, 0.0000];
startPoint = [-0.030, 0.0840, 0.0000];
placementOrderMode = 'csv';

sphereR = 1.10e-3;
allowedSteps = [0.0010, 0.0005, 0.0002, 0.0001];
gridBase     = 0.0001;
safetyMargin = 0e-3;
clearance    = 2 * sphereR + safetyMargin;

transportPathStep = 1.0e-3;
transportClearance = 3.0e-3;
overpassSurfaceGap = 4.0e-3;
enableOverpassTransport = true;
overpassExtraHeights = [0, 1, 2, 3] * 1e-3;
overpassBezierOffsets = [0, 3, -3, 5, -5, 8, -8] * 1e-3;
enableAdaptiveTransport = true;
enableDirectTransportFirst = true;
enableBezierTransport = true;
transportBezierOffsets = [3, -3, 5, -5, 8, -8, 10, -10] * 1e-3;
transportBezierStep = transportPathStep;
longApproachPreferredThreshold = 3.0e-3;
motionMode = 'multi_axis';
enableFinalApproach = true;
approachParentMode = 'nearest_placed_contact';
contactTolerance = 0.35e-3;
finalApproachDistance = 3.0e-3;
finalApproachDistanceCandidates = [6.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.6, 0.4] * 1e-3;
finalApproachStep     = 0.1e-3;
localAdjustStep       = 0.2e-3;
enableTopDownFinalApproach = true;
topDownExtraHeight = 4.0e-3;
topDownStep = 0.1e-3;
multiAxisPathStep = transportPathStep;
finalApproachFailMode = 'fallback_normal';
enableMultiAxisAstar = true;

pathOutputDecimals = 6;

plannerOptions.motionMode = motionMode;
plannerOptions.enableFinalApproach = enableFinalApproach;
plannerOptions.approachParentMode = approachParentMode;
plannerOptions.contactTolerance = contactTolerance;
plannerOptions.finalApproachDistance = finalApproachDistance;
plannerOptions.finalApproachDistanceCandidates = finalApproachDistanceCandidates;
plannerOptions.finalApproachStep = finalApproachStep;
plannerOptions.localAdjustStep = localAdjustStep;
plannerOptions.enableTopDownFinalApproach = enableTopDownFinalApproach;
plannerOptions.topDownExtraHeight = topDownExtraHeight;
plannerOptions.topDownStep = topDownStep;
plannerOptions.multiAxisPathStep = multiAxisPathStep;
plannerOptions.transportPathStep = transportPathStep;
plannerOptions.transportClearance = transportClearance;
plannerOptions.overpassSurfaceGap = overpassSurfaceGap;
plannerOptions.enableOverpassTransport = enableOverpassTransport;
plannerOptions.overpassExtraHeights = overpassExtraHeights;
plannerOptions.overpassBezierOffsets = overpassBezierOffsets;
plannerOptions.enableAdaptiveTransport = enableAdaptiveTransport;
plannerOptions.enableDirectTransportFirst = enableDirectTransportFirst;
plannerOptions.enableBezierTransport = enableBezierTransport;
plannerOptions.transportBezierOffsets = transportBezierOffsets;
plannerOptions.transportBezierStep = transportBezierStep;
plannerOptions.longApproachPreferredThreshold = longApproachPreferredThreshold;
plannerOptions.sphereR = sphereR;
plannerOptions.finalApproachFailMode = finalApproachFailMode;
plannerOptions.enableMultiAxisAstar = enableMultiAxisAstar;

enableAdaptiveSupportRods = true;
enableFirstBallSupportRod = true;
supportRodCount = 3;
supportRodXFractions = [0.50, 0.80];
supportRodSize = [1.5e-3, 1.5e-3, 120e-3];
supportRodTopGap = 0.50e-3;
supportRodNearTopConstantHeightGap = 5.0e-3;
supportRodCrossingMinSurfaceGap    = 3.0e-3;

skeletonToOutlineClearance = 3.0e-3;
enableSkeletonOutlineClearance = true;
enableSkeletonBezierFinalApproach = true;
enableSkeletonArcFinalApproach = true;
skeletonBezierFinalOffsets = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5] * 1e-3;
skeletonArcFinalRadii = [2.0, 3.0, 4.0, 5.0] * 1e-3;

rodControlPointMinSurfaceGap = 3.0e-3;
skeletonBezierRodWeights = [0.5, 1.0, 1.5, 2.0];

enableRodObstacle = enableAdaptiveSupportRods;
rod.center = [0.0000, 0.0825, 0.0000];
rod.size   = supportRodSize;
rod.heightAxis = 2;
rod.longAxis   = 3;
rod.collisionMargin = safetyMargin;
rod.extraHeightGap  = supportRodCrossingMinSurfaceGap;
rod.aboveBufferX = sphereR + safetyMargin;
rod.aboveBufferZ = sphereR + safetyMargin;
rod.verticalTolerance = 1e-9;
rod.targetAboveRodMode = 'allow_lower_target';
rod.aboveRuleMode = 'near_top_band_only';
rod.nearTopConstantHeightGap = supportRodNearTopConstantHeightGap;
rod.crossingMinSurfaceGap = supportRodCrossingMinSurfaceGap;
rod.disableFinalApproachAboveRegion = false;
rod = finalizeRodOptions(rod, sphereR);
plannerOptions.enableRodObstacle = enableRodObstacle;
plannerOptions.rod = rod;
plannerOptions.skeletonToOutlineClearance = skeletonToOutlineClearance;
plannerOptions.enableSkeletonOutlineClearance = enableSkeletonOutlineClearance;
plannerOptions.enableSkeletonBezierFinalApproach = enableSkeletonBezierFinalApproach;
plannerOptions.enableSkeletonArcFinalApproach = enableSkeletonArcFinalApproach;
plannerOptions.skeletonBezierFinalOffsets = skeletonBezierFinalOffsets;
plannerOptions.skeletonArcFinalRadii = skeletonArcFinalRadii;
plannerOptions.rodControlPointMinSurfaceGap = rodControlPointMinSurfaceGap;
plannerOptions.skeletonBezierRodWeights = skeletonBezierRodWeights;

enableRodHeadCurvedApproach = false;
rodHead.enable = enableRodHeadCurvedApproach;
rodHead.side = 'negX';
rodHead.rodSideGap = 0.10e-3;
rodHead.xAnchor = rod.center(1) - rod.size(1)/2 - sphereR - rodHead.rodSideGap;
rodHead.headTol = 0.15e-3;
rodHead.awayDirection = [-1, 0, 0];
rodHead.curveStep = 0.10e-3;
rodHead.entryDistances = [2.0, 3.0, 4.0, 5.0] * 1e-3;
rodHead.controlDistances = [0.8, 1.2, 1.6, 2.0] * 1e-3;
rodHead.directionWeights = [ ...               % [awayRodWeight, awayParentWeight]
    1.0, 0.0;
    1.0, 0.8;
    1.0, 1.2;
    1.5, 1.0;
    2.0, 0.8];
rodHead.rodOnlyDirectionWeights = [1.0, 0.0];
plannerOptions.enableRodHeadCurvedApproach = enableRodHeadCurvedApproach;
plannerOptions.rodHead = rodHead;

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

outFolder = 'ButterflyWing_SUPPORT_RODS_InheritedSpiralLogic_v3_Paths';
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

supportRods = struct([]);
supportBallIDs = [];

if enableAdaptiveSupportRods
    [supportRods, supportBallIDs] = buildAdaptiveSupportRodsFromTargets( ...
        targets, sphereR, supportRodSize, supportRodTopGap, supportRodCount, supportRodXFractions, ...
        safetyMargin, supportRodNearTopConstantHeightGap, supportRodCrossingMinSurfaceGap);

    for rr = 1:numel(supportRods)
        supportRods(rr) = finalizeRodOptions(supportRods(rr), sphereR);
    end

    enableRodObstacle = true;
    plannerOptions.enableRodObstacle = true;
    plannerOptions.supportRods = supportRods;
    plannerOptions.supportBallIDs = supportBallIDs;
    plannerOptions.rod = supportRods(1);
    rod = supportRods(1);

    fprintf('已启用三根支撑长杆：Rod 1 首球支撑，Rod 2/Rod 3 分别位于 Rod 1 左/右侧；三根杆均以 Z=0 为中心对称布置。\n');
    for rr = 1:numel(supportRods)
        fprintf('  Rod %d | role=%s | support globalID=%g | center = [%.3f %.3f %.3f] mm, size = [%.3f %.3f %.3f] mm, Zrange=[%.3f, %.3f] mm, topGap = %.3f mm\n', ...
            rr, supportRods(rr).rodRole, targets(supportBallIDs(rr)).globalID, ...
            supportRods(rr).center(1)*1e3, supportRods(rr).center(2)*1e3, supportRods(rr).center(3)*1e3, ...
            supportRods(rr).size(1)*1e3, supportRods(rr).size(2)*1e3, supportRods(rr).size(3)*1e3, ...
            supportRods(rr).minCorner(3)*1e3, supportRods(rr).maxCorner(3)*1e3, supportRodTopGap*1e3);
    end
else
    enableRodObstacle = false;
    plannerOptions.enableRodObstacle = false;
    plannerOptions.supportRods = struct([]);
    plannerOptions.supportBallIDs = [];
end

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

    thisPlannerOptions = preparePlannerOptionsForCurrentTarget( ...
        plannerOptions, targets, i, placedCenters, placedPointIndices, placedGlobalIDs);

    [path, routeType, stepSummary, approachInfo] = planPathAvoidingObstacles( ...
        startPoint, thisTarget, placedCenters, ...
        sphereR, clearance, allowedSteps, gridBase, ...
        thisPlannerOptions, placedPointIndices, placedGlobalIDs);

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
    points(i).transportOverpassUsed = approachInfo.transportOverpassUsed;
    points(i).transportRouteType = approachInfo.transportRouteType;
    points(i).transportClearance_m = approachInfo.transportClearance_m;
    points(i).transportClearance_mm = approachInfo.transportClearance_m * 1e3;
    points(i).transportMinDistanceToPlaced_m = approachInfo.transportMinDistanceToPlaced_m;
    points(i).transportMinDistanceToPlaced_mm = approachInfo.transportMinDistanceToPlaced_m * 1e3;
    points(i).transportOverpassY_m = approachInfo.transportOverpassY_m;
    points(i).transportOverpassY_mm = approachInfo.transportOverpassY_m * 1e3;
    points(i).transportFailReason = approachInfo.transportFailReason;
    points(i).rodHeadApproachEnabled = approachInfo.rodHeadApproachEnabled;
    points(i).isRodHeadBall = approachInfo.isRodHeadBall;
    points(i).rodHeadApproachUsed = approachInfo.rodHeadApproachUsed;
    points(i).rodHeadApproachType = approachInfo.rodHeadApproachType;
    points(i).rodHeadParentGlobalID = approachInfo.rodHeadParentGlobalID;
    points(i).rodHeadEntryPoint = roundN(approachInfo.rodHeadEntryPoint, pathOutputDecimals);
    points(i).rodHeadEntryPoint_mm = roundN(approachInfo.rodHeadEntryPoint * 1e3, 4);
    points(i).rodHeadControlPoint = roundN(approachInfo.rodHeadControlPoint, pathOutputDecimals);
    points(i).rodHeadControlPoint_mm = roundN(approachInfo.rodHeadControlPoint * 1e3, 4);
    points(i).rodHeadFailReason = approachInfo.rodHeadFailReason;
    points(i).isSkeletonBall = approachInfo.isSkeletonBall;
    points(i).skeletonOutlineClearanceChecked = approachInfo.skeletonOutlineClearanceChecked;
    points(i).skeletonOutlineClearance_m = approachInfo.skeletonOutlineClearance_m;
    points(i).skeletonOutlineClearance_mm = approachInfo.skeletonOutlineClearance_m * 1e3;
    points(i).skeletonOutlineFinalApproachUsed = approachInfo.skeletonOutlineFinalApproachUsed;
    points(i).skeletonOutlineFinalApproachType = approachInfo.skeletonOutlineFinalApproachType;
    points(i).skeletonOutlineFailReason = approachInfo.skeletonOutlineFailReason;
    points(i).skeletonFinalOutlineMinDistance_m = approachInfo.skeletonFinalOutlineMinDistance_m;
    points(i).skeletonFinalOutlineMinDistance_mm = approachInfo.skeletonFinalOutlineMinDistance_m * 1e3;
    points(i).skeletonFinalOutlineNearestGlobalID = approachInfo.skeletonFinalOutlineNearestGlobalID;
    points(i).rodControlPointClearanceChecked = approachInfo.rodControlPointClearanceChecked;
    points(i).rodControlPointMinGap_m = approachInfo.rodControlPointMinGap_m;
    points(i).rodControlPointMinGap_mm = approachInfo.rodControlPointMinGap_m * 1e3;
    points(i).rodControlPointNearestRodID = approachInfo.rodControlPointNearestRodID;
    points(i).rodControlPointFailReason = approachInfo.rodControlPointFailReason;

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

root.description = ['蝴蝶翅膀小球结构逐点打印拼接避碰路径规划；' ...
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
root.finalApproachDistanceCandidates_m = finalApproachDistanceCandidates;
root.finalApproachStep_m = finalApproachStep;
root.enableTopDownFinalApproach = enableTopDownFinalApproach;
root.topDownExtraHeight_m = topDownExtraHeight;
root.topDownStep_m = topDownStep;
root.localAdjustStep_m = localAdjustStep;
root.multiAxisPathStep_m = multiAxisPathStep;
root.transportPathStep_m = transportPathStep;
root.transportClearance_m = transportClearance;
root.overpassSurfaceGap_m = overpassSurfaceGap;
root.enableOverpassTransport = enableOverpassTransport;
root.overpassExtraHeights_m = overpassExtraHeights;
root.overpassBezierOffsets_m = overpassBezierOffsets;
root.enableAdaptiveTransport = enableAdaptiveTransport;
root.enableDirectTransportFirst = enableDirectTransportFirst;
root.enableBezierTransport = enableBezierTransport;
root.transportBezierOffsets_m = transportBezierOffsets;
root.transportBezierStep_m = transportBezierStep;
root.longApproachPreferredThreshold_m = longApproachPreferredThreshold;
root.transportRule = '普通运输段尽量使用 1 mm 步进；自适应策略优先直接斜线到预接近点，其次使用平滑 Bezier 绕开已拼小球，最后才上方跨越；只有路径在已有小球附近低位通过或距离不足时才强制升高。最后贴近与局部下降段只按真实碰撞距离检查。';
root.finalApproachFailMode = finalApproachFailMode;
root.enableMultiAxisAstar = enableMultiAxisAstar;
root.pathOutputDecimals = pathOutputDecimals;
root.enableRodObstacle = enableRodObstacle;
root.enableFirstBallSupportRod = enableFirstBallSupportRod;
root.supportRodSize_m = supportRodSize;
root.supportRodTopGap_m = supportRodTopGap;
root.supportRodNearTopConstantHeightGap_m = supportRodNearTopConstantHeightGap;
root.supportRodCrossingMinSurfaceGap_m = supportRodCrossingMinSurfaceGap;
root.supportRodRule = '三根支撑杆尺寸为1.5 mm × 1.5 mm × 120 mm，均以Z=0为中心对称布置，覆盖Z=-60~+60 mm；Rod1强制位于第1个打印小球下方，Rod2/Rod3分别在Rod1左/右侧自适应支撑；路径不能碰撞支撑杆；普通跨杆路径小球最低表面距杆上表面必须>=3 mm；若当前目标点位于支撑杆正上方，则仅终端局部接近段允许低位进入目标点；对应支撑小球最终目标点允许0.5 mm支撑间隙；preApproach/Bezier/圆弧控制点必须离支撑杆表面>=3 mm。';
root.rodObstacle = rod;
root.supportRods = getPlannerRodsForPlot(plannerOptions);
root.supportBallIDs = getfieldWithDefault(plannerOptions, 'supportBallIDs', []);
root.supportRodCount = numel(getPlannerRodsForPlot(plannerOptions));
root.enableSkeletonOutlineClearance = enableSkeletonOutlineClearance;
root.skeletonToOutlineClearance_m = skeletonToOutlineClearance;
root.enableSkeletonBezierFinalApproach = enableSkeletonBezierFinalApproach;
root.enableSkeletonArcFinalApproach = enableSkeletonArcFinalApproach;
root.skeletonBezierFinalOffsets_m = skeletonBezierFinalOffsets;
root.skeletonArcFinalRadii_m = skeletonArcFinalRadii;
root.rodControlPointMinSurfaceGap_m = rodControlPointMinSurfaceGap;
root.skeletonBezierRodWeights = skeletonBezierRodWeights;
root.skeletonOutlineRule = '内部骨架小球移动路径对非豁免外轮廓球保持>=3 mm球心距；若普通直线最后正面贴近无法满足该约束，或preApproach/Bezier/圆弧控制点离支撑杆不足3 mm，则自动尝试Bezier/圆弧式局部贴近。';
root.rodRule = '路径经过长杆正上方区域时，Y 方向高度必须保持不变；普通小球还必须满足 safeY 高度，X/Z 方向水平移动允许。';
root.enableRodHeadCurvedApproach = enableRodHeadCurvedApproach;
root.rodHeadCurvedApproach = rodHead;
root.rodHeadCurvedApproachRule = '贴杆头部小球不使用普通直线最后贴近；优先从外侧曲线入口点经二次 Bezier 曲线接近目标点，避免中途先粘长杆，并在最后一步完成贴杆/贴球。';
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
    single.description = '蝴蝶翅膀小球结构单点避碰路径规划文件';
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
    single.localAdjustStep_m = localAdjustStep;
    single.multiAxisPathStep_m = multiAxisPathStep;
    single.transportPathStep_m = transportPathStep;
    single.transportClearance_m = transportClearance;
    single.overpassSurfaceGap_m = overpassSurfaceGap;
    single.enableOverpassTransport = enableOverpassTransport;
    single.enableAdaptiveTransport = enableAdaptiveTransport;
    single.enableDirectTransportFirst = enableDirectTransportFirst;
    single.enableBezierTransport = enableBezierTransport;
    single.transportBezierOffsets_m = transportBezierOffsets;
    single.enableRodObstacle = enableRodObstacle;
    single.enableFirstBallSupportRod = enableFirstBallSupportRod;
    single.supportRodSize_m = supportRodSize;
    single.supportRodTopGap_m = supportRodTopGap;
    single.supportRodNearTopConstantHeightGap_m = supportRodNearTopConstantHeightGap;
    single.supportRodCrossingMinSurfaceGap_m = supportRodCrossingMinSurfaceGap;
    single.rodObstacle = rod;
    single.supportRods = getPlannerRodsForPlot(plannerOptions);
    single.supportBallIDs = getfieldWithDefault(plannerOptions, 'supportBallIDs', []);
    single.enableSkeletonOutlineClearance = enableSkeletonOutlineClearance;
    single.skeletonToOutlineClearance_m = skeletonToOutlineClearance;
    single.enableSkeletonBezierFinalApproach = enableSkeletonBezierFinalApproach;
    single.enableSkeletonArcFinalApproach = enableSkeletonArcFinalApproach;
    single.rodControlPointMinSurfaceGap_m = rodControlPointMinSurfaceGap;
    single.skeletonBezierRodWeights = skeletonBezierRodWeights;
    single.enableRodHeadCurvedApproach = enableRodHeadCurvedApproach;
    single.rodHeadCurvedApproach = rodHead;
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
    title('蝴蝶翅膀小球结构逐点打印拼接避碰路径规划');

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
    %     drawRodObstacles(getPlannerRodsForPlot(plannerOptions));
    % end

    if plotAllPaths
        nPlot = min(numel(points), maxPathsToPlot);
        if exist('turbo', 'file') == 2
            cmap = turbo(max(2, nPlot));
        else
            cmap = lines(max(2, nPlot));
        end
        for i = 1:nPlot
            p = points(i).path;
            col = cmap(i,:);
            plot3(p(:,1), p(:,2), p(:,3), 'Color', col, 'LineWidth', 0.75);

            fp = points(i).finalApproachPath;
            if ~isempty(fp) && size(fp,1) >= 2
                plot3(fp(:,1), fp(:,2), fp(:,3), 'Color', col, 'LineWidth', 2.8);
                pp = points(i).preApproachPoint;
                if all(isfinite(pp))
                    scatter3(pp(1), pp(2), pp(3), 18, col, 'filled');
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
    info.transportOverpassUsed = false;
    info.transportRouteType = '';
    info.transportClearance_m = NaN;
    info.transportMinDistanceToPlaced_m = NaN;
    info.transportOverpassY_m = NaN;
    info.transportFailReason = '';
    info.rodHeadApproachEnabled = false;
    info.isRodHeadBall = false;
    info.rodHeadApproachUsed = false;
    info.rodHeadApproachType = '';
    info.rodHeadParentGlobalID = NaN;
    info.rodHeadEntryPoint = [NaN, NaN, NaN];
    info.rodHeadControlPoint = [NaN, NaN, NaN];
    info.rodHeadFailReason = '';
    info.isSkeletonBall = false;
    info.skeletonOutlineClearanceChecked = false;
    info.skeletonOutlineClearance_m = NaN;
    info.skeletonOutlineFinalApproachUsed = false;
    info.skeletonOutlineFinalApproachType = '';
    info.skeletonOutlineFailReason = '';
    info.skeletonFinalOutlineMinDistance_m = NaN;
    info.skeletonFinalOutlineNearestGlobalID = NaN;
    info.rodControlPointClearanceChecked = false;
    info.rodControlPointMinGap_m = NaN;
    info.rodControlPointNearestRodID = NaN;
    info.rodControlPointFailReason = '';
end

function [bestPath, bestRouteType, bestSummary, approachInfo] = planMultiAxisPathWithOptionalFinalApproach( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
    plannerOptions, placedPointIndices, placedGlobalIDs)

    approachInfo = defaultApproachInfo();
    approachInfo.motionMode = 'multi_axis';
    approachInfo = fillRodInfoForTarget(approachInfo, targetPt, plannerOptions);
    approachInfo = fillStartHorizontalInfo(approachInfo, plannerOptions);

    rodAboveTarget = isTargetAboveRodForHorizontalEntry(targetPt, plannerOptions);
    rodHeadTarget = isRodHeadBallForCurvedApproach(targetPt, plannerOptions);
    approachInfo.rodHeadApproachEnabled = isRodHeadCurvedApproachEnabled(plannerOptions);
    approachInfo.isRodHeadBall = rodHeadTarget;

    if rodHeadTarget
        [okHead, headPath, headRouteType, headSummary, headInfo] = planRodHeadCurvedFinalApproach( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
            plannerOptions, placedPointIndices, placedGlobalIDs, approachInfo);
        approachInfo = headInfo;
        if okHead
            bestPath = headPath;
            bestRouteType = headRouteType;
            bestSummary = headSummary;
            return;
        end
    end

    activeRodForTarget = getActiveRodForPoint(plannerOptions, targetPt);
    disableFinalAboveRod = rodAboveTarget && isRodObstacleEnabled(plannerOptions) && ...
        ~isempty(activeRodForTarget) && ...
        (~isfield(activeRodForTarget, 'disableFinalApproachAboveRegion') || activeRodForTarget.disableFinalApproachAboveRegion);
    useFinal = isfield(plannerOptions, 'enableFinalApproach') && plannerOptions.enableFinalApproach && ...
        ~isempty(obstacles) && ~disableFinalAboveRod && ~rodHeadTarget;

    finalTried = false;

    if useFinal
        finalTried = true;
        [okFinal, finalPathAll, finalRouteType, finalSummary, finalInfo] = planNormalFinalApproachCandidates( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
            plannerOptions, placedPointIndices, placedGlobalIDs, approachInfo);
        approachInfo = finalInfo;
        if okFinal
            bestPath = finalPathAll;
            bestRouteType = finalRouteType;
            bestSummary = finalSummary;
            return;
        end

        if isCurrentSkeletonBall(plannerOptions) && isfield(plannerOptions, 'enableSkeletonBezierFinalApproach') && plannerOptions.enableSkeletonBezierFinalApproach
            [okSkelCurve, skelCurvePath, skelCurveRouteType, skelCurveSummary, skelCurveInfo] = planSkeletonCurvedFinalApproach( ...
                startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
                plannerOptions, placedPointIndices, placedGlobalIDs, approachInfo);
            approachInfo = skelCurveInfo;
            if okSkelCurve
                bestPath = skelCurvePath;
                bestRouteType = skelCurveRouteType;
                bestSummary = skelCurveSummary;
                return;
            end
        end

        if isfield(plannerOptions, 'finalApproachFailMode') && strcmpi(plannerOptions.finalApproachFailMode, 'error')
            error('最后正面贴近路径规划失败：%s', approachInfo.failReason);
        end
    else
        if isempty(obstacles)
            approachInfo.failReason = '第一个球无已放置接触球，直接使用普通多轴路径';
        else
            approachInfo.failReason = 'enableFinalApproach = false 或目标属于特殊区域，直接使用普通多轴路径/后续兜底策略';
        end
    end

    if ~rodHeadTarget && isfield(plannerOptions, 'enableTopDownFinalApproach') && ...
            plannerOptions.enableTopDownFinalApproach && ~isempty(obstacles)
        [okTop, topPath, topRouteType, topSummary, topInfo] = planTopDownFinalApproach( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, approachInfo);
        approachInfo = topInfo;
        if okTop
            bestPath = topPath;
            bestRouteType = topRouteType;
            bestSummary = topSummary;
            return;
        end
    end

    if isempty(obstacles) || ~finalTried
        [okNormal, bestPath, bestRouteType, bestSummary] = planSafeOverpassTransportPath( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);

        if okNormal
            bestRouteType = sprintf('%s + no_final_front_approach', bestRouteType);
            bestSummary.finalApproachRouteType = 'not_used';
            approachInfo.globalRouteType = bestRouteType;
            approachInfo = fillTransportInfoFromSummary(approachInfo, bestRouteType, bestSummary, plannerOptions);
            approachInfo = updateStartHorizontalInfoFromRoute(approachInfo, bestRouteType, plannerOptions);
            if contains(lower(bestRouteType), 'rod_safe') || contains(lower(bestRouteType), 'rod_above')
                approachInfo.rodCrossingUsed = true;
                approachInfo.rodRouteType = bestRouteType;
            end
            return;
        end
    end

    error(['路径规划失败：普通多父球/多距离正面贴近、上方垂直下降拼接以及普通运输均未找到可行解。' ...
           '最后失败原因：%s'], approachInfo.failReason);
end

function [ok, bestPath, bestRouteType, bestSummary, info] = planNormalFinalApproachCandidates( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
    plannerOptions, placedPointIndices, placedGlobalIDs, info)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    parentCandidates = findApproachParentCandidates( ...
        targetPt, obstacles, placedPointIndices, placedGlobalIDs, clearance, plannerOptions);

    if isempty(parentCandidates)
        info.failReason = '未找到合适的已放置接触球作为最后正面贴近对象';
        return;
    end

    if isfield(plannerOptions, 'finalApproachDistanceCandidates') && ...
            ~isempty(plannerOptions.finalApproachDistanceCandidates)
        distCandidates = plannerOptions.finalApproachDistanceCandidates(:)';
    else
        distCandidates = plannerOptions.finalApproachDistance;
    end

    feasible = {};
    cnt = 0;
    lastFail = '';

    for pi = 1:numel(parentCandidates)
        parent = parentCandidates(pi);
        dirVec = targetPt - parent.center;
        dirNorm = norm(dirVec);
        if dirNorm <= 1e-12
            lastFail = 'parentCenter 与 targetPt 距离过小，无法定义正面贴近方向';
            continue;
        end

        u = dirVec / dirNorm;
        activeRod = getActiveRodForPoint(plannerOptions, targetPt);
        if isRodObstacleEnabled(plannerOptions) && ~isempty(activeRod) && isTargetAboveRodRegion(targetPt, activeRod)
            u2 = u;
            u2(activeRod.heightAxis) = 0;
            if norm(u2) < 1e-12
                u2 = [1, 0, 0];
            else
                u2 = u2 / norm(u2);
            end
            u = u2;
        end

        for di = 1:numel(distCandidates)
            dApproach = distCandidates(di);
            preApproachPoint = targetPt + dApproach * u;

            [okCtrl, ctrlGap, ctrlRodIdx, ctrlReason] = checkRodControlPointClearance(preApproachPoint, plannerOptions);
            if ~okCtrl
                lastFail = sprintf('preApproachPoint 离支撑杆过近：rod %d, surfaceGap %.3f mm, %s', ...
                    ctrlRodIdx, ctrlGap*1e3, ctrlReason);
                continue;
            end

            finalPath = buildLinePath(preApproachPoint, targetPt, plannerOptions.finalApproachStep);

            if ~isPathCollisionFree(finalPath, obstacles, clearance, plannerOptions, targetPt)
                [minD, nearIdx] = minDistancePathToObstacles(finalPath, obstacles);
                if ~isempty(nearIdx) && nearIdx >= 1 && nearIdx <= numel(placedGlobalIDs)
                    lastFail = sprintf('finalPath 碰撞或过近：parent globalID %.0f, approach %.1f mm, 最近 globalID %.0f, minDist %.3f mm', ...
                        parent.globalID, dApproach*1e3, placedGlobalIDs(nearIdx), minD*1e3);
                else
                    lastFail = sprintf('finalPath 碰撞或过近：parent globalID %.0f, approach %.1f mm', ...
                        parent.globalID, dApproach*1e3);
                end
                continue;
            end

            [okSkelFinal, skelMinD, skelNearGID, skelReason] = validateSkeletonFinalApproachOutlineClearance(finalPath, plannerOptions, targetPt);
            if ~okSkelFinal
                lastFail = sprintf('内部骨架 finalApproachPath 到外轮廓球距离不足：nearest outline globalID %.0f, minDist %.3f mm, %s', ...
                    skelNearGID, skelMinD*1e3, skelReason);
                continue;
            end

            [okGlobal, globalPath, globalRouteType, globalSummary] = planAdaptiveTransportToStaging( ...
                startPt, preApproachPoint, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);

            if ~okGlobal
                lastFail = sprintf('finalPath 可行，但自适应运输到 preApproachPoint 失败：parent globalID %.0f, approach %.1f mm', ...
                    parent.globalID, dApproach*1e3);
                continue;
            end

            path = concatPaths(globalPath, finalPath);
            summary = summarizeMultiAxisPath(path);
            summary.globalRouteType = globalRouteType;
            summary.finalApproachRouteType = sprintf('front_approach_%s_d%.1fmm', parent.modeUsed, dApproach*1e3);
            if isfield(globalSummary, 'transportOverpassUsed')
                summary.transportOverpassUsed = globalSummary.transportOverpassUsed;
            end
            if isfield(globalSummary, 'transportOverpassY_m')
                summary.transportOverpassY_m = globalSummary.transportOverpassY_m;
            end
            if isfield(globalSummary, 'transportClearance_m')
                summary.transportClearance_m = globalSummary.transportClearance_m;
            end
            if isfield(globalSummary, 'transportMinDistanceToPlaced_m')
                summary.transportMinDistanceToPlaced_m = globalSummary.transportMinDistanceToPlaced_m;
            end
            if isfield(globalSummary, 'transportFailReason')
                summary.transportFailReason = globalSummary.transportFailReason;
            end

            candInfo = info;
            candInfo.parentIndex = parent.index;
            candInfo.parentGlobalID = parent.globalID;
            candInfo.parentDistance = parent.distance;
            candInfo.approachDirection = u;
            candInfo.preApproachPoint = preApproachPoint;
            candInfo.finalApproachRouteType = summary.finalApproachRouteType;
            candInfo.finalApproachUsed = true;
            candInfo.finalApproachPath = finalPath;
            candInfo.globalRouteType = globalRouteType;
            candInfo.failReason = '';
            candInfo.skeletonFinalOutlineMinDistance_m = skelMinD;
            candInfo.skeletonFinalOutlineNearestGlobalID = skelNearGID;
            candInfo.rodControlPointClearanceChecked = true;
            candInfo.rodControlPointMinGap_m = ctrlGap;
            candInfo.rodControlPointNearestRodID = ctrlRodIdx;
            candInfo.rodControlPointFailReason = '';
            candInfo = fillTransportInfoFromSummary(candInfo, globalRouteType, globalSummary, plannerOptions);
            candInfo = updateStartHorizontalInfoFromRoute(candInfo, globalRouteType, plannerOptions);
            if contains(lower(globalRouteType), 'rod_safe') || contains(lower(globalRouteType), 'rod_above')
                candInfo.rodCrossingUsed = true;
                candInfo.rodRouteType = globalRouteType;
            end

            cnt = cnt + 1;
            feasible{cnt}.path = path; %#ok<AGROW>
            feasible{cnt}.routeType = sprintf('%s + %s', globalRouteType, summary.finalApproachRouteType); %#ok<AGROW>
            feasible{cnt}.summary = summary; %#ok<AGROW>
            feasible{cnt}.info = candInfo; %#ok<AGROW>
            feasible{cnt}.approachDistance = dApproach; %#ok<AGROW>
            feasible{cnt}.routePriority = getTransportRoutePriority(globalRouteType); %#ok<AGROW>
            feasible{cnt}.parentDist = parent.distance; %#ok<AGROW>
        end
    end

    if isempty(feasible)
        info.failReason = sprintf('所有接触父球和 finalApproachDistanceCandidates 均失败。%s', lastFail);
        return;
    end

    bestIdx = 1;
    bestScore = finalApproachCandidateScore(feasible{1}, plannerOptions);
    for ci = 2:numel(feasible)
        thisScore = finalApproachCandidateScore(feasible{ci}, plannerOptions);
        if lexicographicScoreBetter(thisScore, bestScore)
            bestIdx = ci;
            bestScore = thisScore;
        end
    end

    chosen = feasible{bestIdx};
    ok = true;
    bestPath = chosen.path;
    bestRouteType = chosen.routeType;
    bestSummary = chosen.summary;
    info = chosen.info;
end

function parentCandidates = findApproachParentCandidates( ...
    targetPt, obstacles, placedPointIndices, placedGlobalIDs, clearance, plannerOptions)

    parentCandidates = struct('center', {}, 'index', {}, 'globalID', {}, 'distance', {}, 'modeUsed', {});
    if isempty(obstacles)
        return;
    end

    mode = lower(strtrim(plannerOptions.approachParentMode));
    dists = sqrt(sum((obstacles - targetPt).^2, 2));

    switch mode
        case 'previous_placed'
            idxList = size(obstacles, 1);
        case {'nearest_placed_contact', 'auto'}
            tol = plannerOptions.contactTolerance;
            idxList = find(abs(dists - clearance) <= tol);
            if isempty(idxList)
                return;
            end
            err = abs(dists(idxList) - clearance);
            [~, localOrder] = sortrows([err, dists(idxList)], [1, 2]);
            idxList = idxList(localOrder);
        otherwise
            error('未知 approachParentMode：%s。', plannerOptions.approachParentMode);
    end

    for k = 1:numel(idxList)
        idx = idxList(k);
        parentCandidates(k).center = obstacles(idx,:); %#ok<AGROW>
        if idx <= numel(placedPointIndices)
            parentCandidates(k).index = placedPointIndices(idx); %#ok<AGROW>
        else
            parentCandidates(k).index = idx; %#ok<AGROW>
        end
        if idx <= numel(placedGlobalIDs)
            parentCandidates(k).globalID = placedGlobalIDs(idx); %#ok<AGROW>
        else
            parentCandidates(k).globalID = parentCandidates(k).index; %#ok<AGROW>
        end
        parentCandidates(k).distance = dists(idx); %#ok<AGROW>
        parentCandidates(k).modeUsed = mode; %#ok<AGROW>
    end
end

function [ok, bestPath, bestRouteType, bestSummary, info] = planTopDownFinalApproach( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, info)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    if ~isfield(plannerOptions, 'enableTopDownFinalApproach') || ~plannerOptions.enableTopDownFinalApproach
        return;
    end
    if isempty(obstacles)
        return;
    end

    hAxis = 2;
    if isfield(plannerOptions, 'rod') && isfield(plannerOptions.rod, 'heightAxis')
        hAxis = plannerOptions.rod.heightAxis;
    end
    if ~isfield(plannerOptions, 'topDownExtraHeight') || plannerOptions.topDownExtraHeight <= 0
        plannerOptions.topDownExtraHeight = 4.0e-3;
    end
    if ~isfield(plannerOptions, 'topDownStep') || plannerOptions.topDownStep <= 0
        plannerOptions.topDownStep = plannerOptions.finalApproachStep;
    end
    if ~isfield(plannerOptions, 'sphereR') || plannerOptions.sphereR <= 0
        plannerOptions.sphereR = clearance / 2;
    end
    if ~isfield(plannerOptions, 'overpassSurfaceGap') || plannerOptions.overpassSurfaceGap < 0
        plannerOptions.overpassSurfaceGap = 0;
    end

    yOver = max(obstacles(:,hAxis)) + 2 * plannerOptions.sphereR + plannerOptions.overpassSurfaceGap;
    yTop = max([targetPt(hAxis) + 2 * plannerOptions.sphereR + plannerOptions.topDownExtraHeight, yOver]);
    aboveTarget = targetPt;
    aboveTarget(hAxis) = yTop;

    [okGlobal, globalPath, globalRouteType, globalSummary] = planSafeOverpassTransportPath( ...
        startPt, aboveTarget, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);
    if ~okGlobal
        info.failReason = 'top_down_final_approach 失败：无法安全运输到目标点正上方';
        return;
    end

    descendPath = buildLinePath(aboveTarget, targetPt, plannerOptions.topDownStep);

    if ~isTopDownDescentPathCollisionFree(descendPath, obstacles, clearance, plannerOptions, targetPt)
        [minD, nearIdx] = minDistancePathToObstacles(descendPath, obstacles);
        if ~isempty(nearIdx) && nearIdx >= 1 && minD < clearance - 1e-12
            info.failReason = sprintf('top_down_final_approach 失败：垂直下降段碰撞，最近障碍序号 %d，minDist %.3f mm', nearIdx, minD*1e3);
        else
            info.failReason = 'top_down_final_approach 失败：垂直下降段与支撑杆规则或内部骨架-外轮廓安全距离规则冲突';
        end
        return;
    end

    bestPath = concatPaths(globalPath, descendPath);
    bestRouteType = sprintf('%s + top_down_final_approach', globalRouteType);
    bestSummary = summarizeMultiAxisPath(bestPath);
    bestSummary.globalRouteType = globalRouteType;
    bestSummary.finalApproachRouteType = 'top_down_final_approach';
    if isfield(globalSummary, 'transportOverpassUsed')
        bestSummary.transportOverpassUsed = globalSummary.transportOverpassUsed;
    end
    if isfield(globalSummary, 'transportOverpassY_m')
        bestSummary.transportOverpassY_m = globalSummary.transportOverpassY_m;
    end
    if isfield(globalSummary, 'transportClearance_m')
        bestSummary.transportClearance_m = globalSummary.transportClearance_m;
    end
    if isfield(globalSummary, 'transportMinDistanceToPlaced_m')
        bestSummary.transportMinDistanceToPlaced_m = globalSummary.transportMinDistanceToPlaced_m;
    end
    bestSummary.transportFailReason = '';
    info.finalApproachUsed = true;
    info.finalApproachPath = descendPath;
    info.preApproachPoint = aboveTarget;
    info.globalRouteType = globalRouteType;
    info.finalApproachRouteType = 'top_down_final_approach';
    info.approachDirection = [0, -1, 0];
    info.failReason = '';
    info = fillTransportInfoFromSummary(info, globalRouteType, globalSummary, plannerOptions);
    info = updateStartHorizontalInfoFromRoute(info, globalRouteType, plannerOptions);
    ok = true;
end

function ok = isTopDownDescentPathCollisionFree(path, obstacles, clearance, plannerOptions, targetPt)
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

    contactTol = 0;
    if isfield(plannerOptions, 'contactTolerance') && plannerOptions.contactTolerance > 0
        contactTol = plannerOptions.contactTolerance;
    end

    for ii = 2:size(path,1)
        p0 = path(ii-1,:);
        p1 = path(ii,:);

        if ~isempty(obstacles)
            for jj = 1:size(obstacles,1)
                c = obstacles(jj,:);
                dTarget = norm(c - targetPt);
                isTargetContactNeighbor = dTarget <= clearance + contactTol + 1e-12;
                dSeg = pointToSegmentDistance(c, p0, p1);
                if isTargetContactNeighbor

                    relaxedClearance = clearance - min(contactTol, 0.2e-3);
                    if dSeg < relaxedClearance - 1e-12
                        ok = false;
                        return;
                    end
                else
                    if dSeg < clearance - 1e-12
                        ok = false;
                        return;
                    end
                end
            end
        end

        if ~segmentIsValidWithRod(p0, p1, plannerOptions, targetPt)
            ok = false;
            return;
        end

        if ~segmentSatisfiesSkeletonOutlineClearance(p0, p1, plannerOptions, targetPt)
            ok = false;
            return;
        end
    end
end

function [minD, nearIdx] = minDistancePathToObstacles(path, obstacles)
    minD = inf;
    nearIdx = [];
    if isempty(obstacles) || size(path,1) < 2
        return;
    end
    for ii = 2:size(path,1)
        a = path(ii-1,:);
        b = path(ii,:);
        for jj = 1:size(obstacles,1)
            d = pointToSegmentDistance(obstacles(jj,:), a, b);
            if d < minD
                minD = d;
                nearIdx = jj;
            end
        end
    end
end

function tf = isRodHeadCurvedApproachEnabled(plannerOptions)
    tf = isfield(plannerOptions, 'enableRodHeadCurvedApproach') && plannerOptions.enableRodHeadCurvedApproach && ...
         isfield(plannerOptions, 'rodHead') && isfield(plannerOptions.rodHead, 'enable') && plannerOptions.rodHead.enable;
end

function tf = isRodHeadBallForCurvedApproach(targetPt, plannerOptions)
    tf = false;
    if ~isRodHeadCurvedApproachEnabled(plannerOptions)
        return;
    end

    head = plannerOptions.rodHead;
    if ~isfield(head, 'xAnchor') || ~isfield(head, 'headTol')
        return;
    end

    tf = abs(targetPt(1) - head.xAnchor) <= head.headTol;
end

function [ok, bestPath, bestRouteType, bestSummary, info] = planRodHeadCurvedFinalApproach( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
    plannerOptions, placedPointIndices, placedGlobalIDs, info)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];
    info.rodHeadApproachEnabled = true;
    info.isRodHeadBall = true;
    info.rodHeadApproachUsed = false;
    info.rodHeadFailReason = '';

    if ~isRodHeadCurvedApproachEnabled(plannerOptions)
        info.rodHeadFailReason = '未启用 rodHeadCurvedApproach';
        return;
    end

    [hasParent, parentCenter, parentIndex, parentGlobalID, parentDist, parentMode] = findApproachParent( ...
        targetPt, obstacles, placedPointIndices, placedGlobalIDs, clearance, plannerOptions);

    curveCandidates = buildRodHeadBezierCandidates(targetPt, parentCenter, hasParent, plannerOptions);
    if isempty(curveCandidates)
        info.rodHeadFailReason = '未生成可用的贴杆头部曲线候选';
        return;
    end

    feasible = {};
    cnt = 0;

    for ci = 1:numel(curveCandidates)
        finalPath = curveCandidates{ci}.path;
        arcStart = finalPath(1,:);

        if ~isPathCollisionFree(finalPath, obstacles, clearance, plannerOptions, targetPt)
            continue;
        end

        [okGlobal, globalPath, globalRouteType, globalSummary] = planAdaptiveTransportToStaging( ...
            startPt, arcStart, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);

        if ~okGlobal
            continue;
        end

        path = concatPaths(globalPath, finalPath);
        summary = summarizeMultiAxisPath(path);
        summary.globalRouteType = globalRouteType;
        summary.finalApproachRouteType = curveCandidates{ci}.type;
        summary.transportSummary = globalSummary;
        cnt = cnt + 1;
        feasible{cnt}.path = path; %#ok<AGROW>
        feasible{cnt}.summary = summary; %#ok<AGROW>
        feasible{cnt}.globalSummary = globalSummary; %#ok<AGROW>
        feasible{cnt}.globalRouteType = globalRouteType; %#ok<AGROW>
        feasible{cnt}.candidate = curveCandidates{ci}; %#ok<AGROW>
    end

    if isempty(feasible)
        if hasParent
            info.rodHeadFailReason = sprintf('贴杆头部曲线接近失败；存在父球 globalID %.0f，parentDist=%.3f mm，但所有曲线候选或入口路径均不合法', parentGlobalID, parentDist*1e3);
        else
            info.rodHeadFailReason = '贴杆头部曲线接近失败；该点为 rod-only 锚定点，但所有曲线候选或入口路径均不合法';
        end
        return;
    end

    bestIdx = 1;
    bestSteps = feasible{1}.summary.totalSteps;
    bestLen = feasible{1}.summary.totalLength_m;
    for ci = 2:numel(feasible)
        s = feasible{ci}.summary.totalSteps;
        L = feasible{ci}.summary.totalLength_m;
        if (s < bestSteps) || (s == bestSteps && L < bestLen)
            bestIdx = ci;
            bestSteps = s;
            bestLen = L;
        end
    end

    chosen = feasible{bestIdx};
    bestPath = chosen.path;
    bestSummary = chosen.summary;
    bestRouteType = sprintf('%s + %s', chosen.globalRouteType, chosen.candidate.type);
    info.finalApproachUsed = true;
    info.finalApproachPath = chosen.candidate.path;
    info.preApproachPoint = chosen.candidate.path(1,:);
    info.globalRouteType = chosen.globalRouteType;
    info = fillTransportInfoFromSummary(info, chosen.globalRouteType, chosen.globalSummary, plannerOptions);
    info.finalApproachRouteType = chosen.candidate.type;
    info.approachDirection = chosen.candidate.approachDirection;
    info.parentIndex = parentIndex;
    info.parentGlobalID = parentGlobalID;
    info.parentDistance = parentDist;
    info.rodHeadApproachUsed = true;
    info.rodHeadApproachType = chosen.candidate.type;
    info.rodHeadParentGlobalID = parentGlobalID;
    info.rodHeadEntryPoint = chosen.candidate.path(1,:);
    info.rodHeadControlPoint = chosen.candidate.controlPoint;
    info.rodHeadFailReason = '';
    info.failReason = '';
    info = updateStartHorizontalInfoFromRoute(info, chosen.globalRouteType, plannerOptions);
    ok = true;
end

function candidates = buildRodHeadBezierCandidates(targetPt, parentCenter, hasParent, plannerOptions)
    candidates = {};
    head = plannerOptions.rodHead;
    awayRod = head.awayDirection;
    awayRod(plannerOptions.rod.heightAxis) = 0;
    if norm(awayRod) < 1e-12
        awayRod = [-1, 0, 0];
    end
    awayRod = awayRod / norm(awayRod);

    if hasParent
        awayParent = targetPt - parentCenter;
        awayParent(plannerOptions.rod.heightAxis) = 0;
        if norm(awayParent) < 1e-12
            awayParent = [0, 0, 1];
        else
            awayParent = awayParent / norm(awayParent);
        end
        weights = head.directionWeights;
        curveType = 'rod_head_bezier_dual_contact';
    else
        awayParent = [0, 0, 0];
        weights = head.rodOnlyDirectionWeights;
        curveType = 'rod_head_bezier_rod_only';
    end

    cnt = 0;
    for wi = 1:size(weights,1)
        wRod = weights(wi,1);
        wParent = weights(wi,2);
        dirVec = wRod * awayRod + wParent * awayParent;
        dirVec(plannerOptions.rod.heightAxis) = 0;
        if norm(dirVec) < 1e-12
            dirVec = awayRod;
        else
            dirVec = dirVec / norm(dirVec);
        end

        for ei = 1:numel(head.entryDistances)
            entryDist = head.entryDistances(ei);
            for ci = 1:numel(head.controlDistances)
                ctrlDist = head.controlDistances(ci);
                entryDir = awayRod;
                p0 = targetPt + entryDist * entryDir;
                p2 = targetPt;
                p1 = targetPt + ctrlDist * dirVec;
                p0(plannerOptions.rod.heightAxis) = targetPt(plannerOptions.rod.heightAxis);
                p1(plannerOptions.rod.heightAxis) = targetPt(plannerOptions.rod.heightAxis);

                path = buildQuadraticBezierPath(p0, p1, p2, head.curveStep);
                cnt = cnt + 1;
                candidates{cnt}.path = path; %#ok<AGROW>
                candidates{cnt}.controlPoint = p1; %#ok<AGROW>
                candidates{cnt}.approachDirection = dirVec; %#ok<AGROW>
                candidates{cnt}.type = curveType; %#ok<AGROW>
            end
        end
    end
end

function path = buildQuadraticBezierPath(p0, p1, p2, maxStepLen)
    approxLen = norm(p1 - p0) + norm(p2 - p1);
    nSteps = max(6, ceil(approxLen / maxStepLen));
    t = linspace(0, 1, nSteps + 1)';
    path = ((1-t).^2) .* p0 + (2*(1-t).*t) .* p1 + (t.^2) .* p2;
end

function info = fillTransportInfoFromSummary(info, routeType, summary, plannerOptions)
    if nargin < 3 || isempty(summary)
        return;
    end

    info.transportRouteType = routeType;
    if isfield(plannerOptions, 'transportClearance')
        info.transportClearance_m = plannerOptions.transportClearance;
    end
    if isfield(summary, 'transportOverpassUsed')
        info.transportOverpassUsed = summary.transportOverpassUsed;
    end
    if isfield(summary, 'transportMinDistanceToPlaced_m')
        info.transportMinDistanceToPlaced_m = summary.transportMinDistanceToPlaced_m;
    end
    if isfield(summary, 'transportOverpassY_m')
        info.transportOverpassY_m = summary.transportOverpassY_m;
    end
    if isfield(summary, 'transportFailReason')
        info.transportFailReason = summary.transportFailReason;
    end
end

function [ok, path, routeName] = buildRodNearBandConstantYEntryPath(startPt, stagingPt, plannerOptions)
    ok = false;
    path = [];
    routeName = 'rod_near_band_constantY_horizontal_entry';

    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end
    rod = getActiveRodForPoint(plannerOptions, stagingPt);
    if isempty(rod)
        return;
    end
    hAxis = rod.heightAxis;
    sphereR = getRodSphereRadius(rod, plannerOptions);

    if ~isTargetAboveRodRegion(stagingPt, rod)
        return;
    end

    surfaceGap = stagingPt(hAxis) - sphereR - rod.topY;
    if surfaceGap > rod.nearTopConstantHeightGap + 1e-12
        return;
    end

    if ~isfield(plannerOptions, 'transportPathStep') || plannerOptions.transportPathStep <= 0
        stepLen = 1.0e-3;
    else
        stepLen = plannerOptions.transportPathStep;
    end

    margin = max(0.2e-3, 0.2 * sphereR);
    entry = stagingPt;

    if startPt(1) <= rod.center(1)
        entry(1) = rod.aboveMinX - margin;
    else
        entry(1) = rod.aboveMaxX + margin;
    end
    entry(hAxis) = stagingPt(hAxis);
    entry(3) = stagingPt(3);

    path1 = buildLinePath(startPt, entry, stepLen);
    path2 = buildLinePath(entry, stagingPt, stepLen);
    path = concatPaths(path1, path2);
    ok = true;
end

function [ok, bestPath, bestRouteType, bestSummary] = planAdaptiveTransportToStaging( ...
    startPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    if nargin < 8 || isempty(targetPt)
        targetPt = stagingPt;
    end

    if ~isfield(plannerOptions, 'enableAdaptiveTransport') || ~plannerOptions.enableAdaptiveTransport
        [ok, bestPath, bestRouteType, bestSummary] = planSafeOverpassTransportWithLocalDescent( ...
            startPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);
        return;
    end

    if ~isfield(plannerOptions, 'transportPathStep') || plannerOptions.transportPathStep <= 0
        plannerOptions.transportPathStep = plannerOptions.multiAxisPathStep;
    end

    if isStartHorizontalConstraintActive(startPt, plannerOptions)
        [okPrefix, prefixPath, exitPt, failReason] = buildStartHorizontalPrefixPath( ...
            startPt, obstacles, clearance, plannerOptions, targetPt);
        if ~okPrefix
            bestRouteType = sprintf('start_horizontal_failed_%s', failReason);
            bestSummary = emptyTransportSummary(plannerOptions, failReason);
            return;
        end

        plannerOptionsRest = disableStartHorizontalConstraint(plannerOptions);
        [okRest, restPath, restRouteType, restSummary] = planAdaptiveTransportToStaging( ...
            exitPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptionsRest, targetPt);

        if okRest
            bestPath = concatPaths(prefixPath, restPath);
            bestRouteType = sprintf('start_horizontal_%.1fmm + %s', ...
                plannerOptions.startConstraint.forwardDistance * 1e3, restRouteType);
            bestSummary = summarizeMultiAxisPath(bestPath);
            bestSummary.transportOverpassUsed = getSummaryField(restSummary, 'transportOverpassUsed', false);
            bestSummary.transportOverpassY_m = getSummaryField(restSummary, 'transportOverpassY_m', NaN);
            bestSummary.transportClearance_m = getSummaryField(restSummary, 'transportClearance_m', plannerOptions.transportClearance);
            bestSummary.transportMinDistanceToPlaced_m = getSummaryField(restSummary, 'transportMinDistanceToPlaced_m', calculateMinPathDistanceToObstacles(bestPath, obstacles));
            bestSummary.transportFailReason = '';
            bestSummary.adaptiveTransportUsed = true;
            bestSummary.adaptiveTransportMode = restRouteType;
            ok = true;
        else
            bestRouteType = sprintf('start_horizontal_ok_but_adaptive_failed + %s', restRouteType);
            bestSummary = restSummary;
        end
        return;
    end

    feasible = {};
    cnt = 0;
    lastFail = '';

    [okConstY, constYPath, constYName] = buildRodNearBandConstantYEntryPath( ...
        startPt, stagingPt, plannerOptions);
    if okConstY
        [safeConstY, failConstY, minDistConstY] = isAdaptiveTransportPathSafe( ...
            constYPath, obstacles, clearance, plannerOptions, targetPt);
        if safeConstY
            cnt = cnt + 1;
            feasible{cnt}.path = constYPath; %#ok<AGROW>
            feasible{cnt}.name = constYName; %#ok<AGROW>
            feasible{cnt}.summary = summarizeMultiAxisPath(constYPath); %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassUsed = false; %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassY_m = NaN; %#ok<AGROW>
            feasible{cnt}.summary.transportClearance_m = plannerOptions.transportClearance; %#ok<AGROW>
            feasible{cnt}.summary.transportMinDistanceToPlaced_m = minDistConstY; %#ok<AGROW>
            feasible{cnt}.summary.transportFailReason = ''; %#ok<AGROW>
            feasible{cnt}.priority = 0; %#ok<AGROW>
        else
            lastFail = sprintf('constantY rod-near entry failed: %s', failConstY);
        end
    end

    if ~isfield(plannerOptions, 'enableDirectTransportFirst') || plannerOptions.enableDirectTransportFirst
        path = buildLinePath(startPt, stagingPt, plannerOptions.transportPathStep);
        [safe, failReason, minDist] = isAdaptiveTransportPathSafe(path, obstacles, clearance, plannerOptions, targetPt);
        if safe
            cnt = cnt + 1;
            feasible{cnt}.path = path; %#ok<AGROW>
            feasible{cnt}.name = 'direct_transport_1mm_adaptive'; %#ok<AGROW>
            feasible{cnt}.summary = summarizeMultiAxisPath(path); %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassUsed = false; %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassY_m = NaN; %#ok<AGROW>
            feasible{cnt}.summary.transportClearance_m = plannerOptions.transportClearance; %#ok<AGROW>
            feasible{cnt}.summary.transportMinDistanceToPlaced_m = minDist; %#ok<AGROW>
            feasible{cnt}.summary.transportFailReason = ''; %#ok<AGROW>
            feasible{cnt}.priority = 1; %#ok<AGROW>
        else
            lastFail = sprintf('direct failed: %s', failReason);
        end
    end

    if isempty(feasible) && isfield(plannerOptions, 'enableBezierTransport') && plannerOptions.enableBezierTransport
        bezierCandidates = buildAdaptiveTransportBezierCandidates(startPt, stagingPt, obstacles, plannerOptions);
        for bi = 1:numel(bezierCandidates)
            path = bezierCandidates{bi}.path;
            [safe, failReason, minDist] = isAdaptiveTransportPathSafe(path, obstacles, clearance, plannerOptions, targetPt);
            if ~safe
                lastFail = sprintf('bezier failed: %s', failReason);
                continue;
            end
            cnt = cnt + 1;
            feasible{cnt}.path = path; %#ok<AGROW>
            feasible{cnt}.name = sprintf('adaptive_bezier_transport_%s', bezierCandidates{bi}.name); %#ok<AGROW>
            feasible{cnt}.summary = summarizeMultiAxisPath(path); %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassUsed = false; %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassY_m = NaN; %#ok<AGROW>
            feasible{cnt}.summary.transportClearance_m = plannerOptions.transportClearance; %#ok<AGROW>
            feasible{cnt}.summary.transportMinDistanceToPlaced_m = minDist; %#ok<AGROW>
            feasible{cnt}.summary.transportFailReason = ''; %#ok<AGROW>
            feasible{cnt}.priority = 2; %#ok<AGROW>
        end
    end

    if isempty(feasible)
        [okOver, overPath, overRouteType, overSummary] = planSafeOverpassTransportWithLocalDescent( ...
            startPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);
        if okOver
            cnt = cnt + 1;
            feasible{cnt}.path = overPath; %#ok<AGROW>
            feasible{cnt}.name = sprintf('adaptive_fallback_%s', overRouteType); %#ok<AGROW>
            feasible{cnt}.summary = overSummary; %#ok<AGROW>
            feasible{cnt}.priority = 3; %#ok<AGROW>
        else
            lastFail = sprintf('%s；overpass failed: %s', lastFail, getSummaryField(overSummary, 'transportFailReason', 'unknown'));
        end
    end

    if isempty(feasible)
        bestRouteType = 'adaptive_transport_failed';
        bestSummary = emptyTransportSummary(plannerOptions, lastFail);
        return;
    end

    bestIdx = 1;
    for ci = 2:numel(feasible)
        a = feasible{bestIdx};
        b = feasible{ci};
        better = false;
        if b.priority < a.priority
            better = true;
        elseif b.priority == a.priority
            if b.summary.totalSteps < a.summary.totalSteps
                better = true;
            elseif b.summary.totalSteps == a.summary.totalSteps && b.summary.totalLength_m < a.summary.totalLength_m
                better = true;
            elseif b.summary.totalSteps == a.summary.totalSteps && abs(b.summary.totalLength_m - a.summary.totalLength_m) < 1e-12 && ...
                    getSummaryField(b.summary, 'transportMinDistanceToPlaced_m', -inf) > getSummaryField(a.summary, 'transportMinDistanceToPlaced_m', -inf)
                better = true;
            end
        end
        if better
            bestIdx = ci;
        end
    end

    ok = true;
    bestPath = feasible{bestIdx}.path;
    bestRouteType = feasible{bestIdx}.name;
    bestSummary = feasible{bestIdx}.summary;
    bestSummary.adaptiveTransportUsed = true;
    bestSummary.adaptiveTransportMode = bestRouteType;
end

function [safe, failReason, minDist] = isAdaptiveTransportPathSafe(path, obstacles, collisionClearance, plannerOptions, targetPt)
    [safe, failReason, minDist] = isGlobalTransportPathSafe(path, obstacles, collisionClearance, plannerOptions, targetPt);
end

function candidates = buildAdaptiveTransportBezierCandidates(startPt, stagingPt, obstacles, plannerOptions)
    candidates = {};
    if ~isfield(plannerOptions, 'transportBezierStep') || plannerOptions.transportBezierStep <= 0
        plannerOptions.transportBezierStep = plannerOptions.transportPathStep;
    end
    if ~isfield(plannerOptions, 'transportBezierOffsets') || isempty(plannerOptions.transportBezierOffsets)
        plannerOptions.transportBezierOffsets = [3, -3, 5, -5] * 1e-3;
    end

    hAxis = 2;
    if isfield(plannerOptions, 'rod') && isfield(plannerOptions.rod, 'heightAxis')
        hAxis = plannerOptions.rod.heightAxis;
    end

    v = stagingPt - startPt;
    v(hAxis) = 0;
    if norm(v) < 1e-12
        perp = [0, 0, 1];
    else
        perp = [-v(3), 0, v(1)];
        if norm(perp) < 1e-12
            perp = [0, 0, 1];
        else
            perp = perp / norm(perp);
        end
    end

    if ~isempty(obstacles)
        mid = 0.5 * (startPt + stagingPt);
        c = mean(obstacles, 1);
        away = mid - c;
        away(hAxis) = 0;
        if norm(away) > 1e-12
            away = away / norm(away);
            if dot(away, perp) < 0
                perp = -perp;
            end
        end
    end

    offsets = plannerOptions.transportBezierOffsets;
    cnt = 0;
    for oi = 1:numel(offsets)
        off = offsets(oi);
        dir = perp;
        if off < 0
            dir = -perp;
        end
        ctrl = 0.5 * (startPt + stagingPt) + abs(off) * dir;

        ctrl(hAxis) = 0.5 * (startPt(hAxis) + stagingPt(hAxis));
        path = buildQuadraticBezierPath(startPt, ctrl, stagingPt, plannerOptions.transportBezierStep);
        cnt = cnt + 1;
        candidates{cnt}.name = sprintf('offset_%+.1fmm', off*1e3); %#ok<AGROW>
        candidates{cnt}.path = path; %#ok<AGROW>
    end
end

function pr = getTransportRoutePriority(routeType)
    s = lower(char(routeType));
    if contains(s, 'direct_transport')
        pr = 1;
    elseif contains(s, 'adaptive_bezier') || contains(s, 'bezier_transport')
        pr = 2;
    elseif contains(s, 'overpass') || contains(s, 'local_descent')
        pr = 3;
    else
        pr = 4;
    end
end

function score = finalApproachCandidateScore(candidate, plannerOptions)
    if isfield(candidate, 'routePriority')
        routePr = candidate.routePriority;
    else
        routePr = getTransportRoutePriority(candidate.routeType);
    end
    d = candidate.approachDistance;
    threshold = 3.0e-3;
    if isfield(plannerOptions, 'longApproachPreferredThreshold')
        threshold = plannerOptions.longApproachPreferredThreshold;
    end
    shortPenalty = 0;
    if d < threshold
        shortPenalty = 5;
    end

    score = [routePr + shortPenalty, -d, candidate.summary.totalSteps, candidate.summary.totalLength_m];
end

function tf = lexicographicScoreBetter(a, b)
    tf = false;
    n = min(numel(a), numel(b));
    for ii = 1:n
        if a(ii) < b(ii) - 1e-12
            tf = true;
            return;
        elseif a(ii) > b(ii) + 1e-12
            tf = false;
            return;
        end
    end
end

function [ok, bestPath, bestRouteType, bestSummary] = planSafeOverpassTransportWithLocalDescent( ...
    startPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    if nargin < 8 || isempty(targetPt)
        targetPt = stagingPt;
    end

    if isempty(obstacles) || ~isfield(plannerOptions, 'enableOverpassTransport') || ~plannerOptions.enableOverpassTransport
        [ok, bestPath, bestRouteType, bestSummary] = planSafeOverpassTransportPath( ...
            startPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);
        return;
    end

    if ~isfield(plannerOptions, 'sphereR') || plannerOptions.sphereR <= 0
        plannerOptions.sphereR = clearance / 2;
    end
    if ~isfield(plannerOptions, 'overpassSurfaceGap') || plannerOptions.overpassSurfaceGap < 0
        plannerOptions.overpassSurfaceGap = 0;
    end
    if ~isfield(plannerOptions, 'overpassExtraHeights')
        plannerOptions.overpassExtraHeights = 0;
    end
    if ~isfield(plannerOptions, 'localAdjustStep') || plannerOptions.localAdjustStep <= 0
        if isfield(plannerOptions, 'finalApproachStep') && plannerOptions.finalApproachStep > 0
            plannerOptions.localAdjustStep = plannerOptions.finalApproachStep;
        else
            plannerOptions.localAdjustStep = plannerOptions.transportPathStep;
        end
    end

    hAxis = 2;
    if isfield(plannerOptions, 'rod') && isfield(plannerOptions.rod, 'heightAxis')
        hAxis = plannerOptions.rod.heightAxis;
    end

    yOverBase = max(obstacles(:, hAxis)) + 2 * plannerOptions.sphereR + plannerOptions.overpassSurfaceGap;
    yOverBase = max([yOverBase, startPt(hAxis), stagingPt(hAxis)]);
    feasible = {};
    cnt = 0;

    for hi = 1:numel(plannerOptions.overpassExtraHeights)
        yOver = yOverBase + plannerOptions.overpassExtraHeights(hi);
        aboveStagingPt = stagingPt;
        aboveStagingPt(hAxis) = yOver;

        [okHigh, highPath, highRouteType, highSummary] = planSafeOverpassTransportPath( ...
            startPt, aboveStagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);
        if ~okHigh
            continue;
        end

        localPath = buildLinePath(aboveStagingPt, stagingPt, plannerOptions.localAdjustStep);
        if ~isPathCollisionFree(localPath, obstacles, clearance, plannerOptions, targetPt)
            continue;
        end

        path = concatPaths(highPath, localPath);
        summary = summarizeMultiAxisPath(path);
        summary.transportOverpassUsed = true;
        summary.transportOverpassY_m = getSummaryField(highSummary, 'transportOverpassY_m', yOver);
        summary.transportClearance_m = getSummaryField(highSummary, 'transportClearance_m', plannerOptions.transportClearance);
        summary.transportMinDistanceToPlaced_m = getSummaryField(highSummary, 'transportMinDistanceToPlaced_m', calculateMinPathDistanceToObstacles(highPath, obstacles));
        summary.transportFailReason = '';
        summary.localDescentUsed = true;
        summary.localDescentStartPoint = aboveStagingPt;
        summary.localDescentEndPoint = stagingPt;
        cnt = cnt + 1;
        feasible{cnt}.path = path; %#ok<AGROW>
        feasible{cnt}.name = sprintf('%s + local_descent_to_staging', highRouteType); %#ok<AGROW>
        feasible{cnt}.summary = summary; %#ok<AGROW>
    end

    if isempty(feasible)
        bestRouteType = 'safe_overpass_to_above_staging_local_descent_failed';
        bestSummary = emptyTransportSummary(plannerOptions, '高位运输到预接近点上方或局部下降段失败');
        return;
    end

    bestIdx = 1;
    for ci = 2:numel(feasible)
        a = feasible{bestIdx}.summary;
        b = feasible{ci}.summary;
        better = false;
        if b.totalSteps < a.totalSteps
            better = true;
        elseif b.totalSteps == a.totalSteps && b.totalLength_m < a.totalLength_m
            better = true;
        elseif b.totalSteps == a.totalSteps && abs(b.totalLength_m - a.totalLength_m) < 1e-12 && ...
                b.transportMinDistanceToPlaced_m > a.transportMinDistanceToPlaced_m
            better = true;
        end
        if better
            bestIdx = ci;
        end
    end

    ok = true;
    bestPath = feasible{bestIdx}.path;
    bestRouteType = feasible{bestIdx}.name;
    bestSummary = feasible{bestIdx}.summary;
end

function [ok, bestPath, bestRouteType, bestSummary] = planSafeOverpassTransportPath( ...
    startPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    if nargin < 8 || isempty(targetPt)
        targetPt = stagingPt;
    end

    if ~isfield(plannerOptions, 'transportPathStep') || plannerOptions.transportPathStep <= 0
        plannerOptions.transportPathStep = plannerOptions.multiAxisPathStep;
    end
    if ~isfield(plannerOptions, 'transportClearance') || plannerOptions.transportClearance <= 0
        plannerOptions.transportClearance = clearance;
    end
    if ~isfield(plannerOptions, 'sphereR') || plannerOptions.sphereR <= 0
        plannerOptions.sphereR = clearance / 2;
    end
    if ~isfield(plannerOptions, 'overpassSurfaceGap') || plannerOptions.overpassSurfaceGap < 0
        plannerOptions.overpassSurfaceGap = 0;
    end
    if ~isfield(plannerOptions, 'overpassExtraHeights')
        plannerOptions.overpassExtraHeights = 0;
    end
    if ~isfield(plannerOptions, 'overpassBezierOffsets')
        plannerOptions.overpassBezierOffsets = 0;
    end

    if isStartHorizontalConstraintActive(startPt, plannerOptions)
        [okPrefix, prefixPath, exitPt, failReason] = buildStartHorizontalPrefixPath( ...
            startPt, obstacles, clearance, plannerOptions, targetPt);
        if ~okPrefix
            bestRouteType = sprintf('start_horizontal_failed_%s', failReason);
            bestSummary = emptyTransportSummary(plannerOptions, failReason);
            return;
        end

        plannerOptionsRest = disableStartHorizontalConstraint(plannerOptions);
        [okRest, restPath, restRouteType, restSummary] = planSafeOverpassTransportPath( ...
            exitPt, stagingPt, obstacles, clearance, allowedSteps, gridBase, plannerOptionsRest, targetPt);

        if okRest
            bestPath = concatPaths(prefixPath, restPath);
            bestRouteType = sprintf('start_horizontal_%.1fmm + %s', ...
                plannerOptions.startConstraint.forwardDistance * 1e3, restRouteType);
            bestSummary = summarizeMultiAxisPath(bestPath);
            bestSummary.transportOverpassUsed = true;
            if isfield(restSummary, 'transportOverpassUsed')
                bestSummary.transportOverpassUsed = restSummary.transportOverpassUsed;
            end
            bestSummary.transportOverpassY_m = getSummaryField(restSummary, 'transportOverpassY_m', NaN);
            bestSummary.transportClearance_m = plannerOptions.transportClearance;
            bestSummary.transportMinDistanceToPlaced_m = getSummaryField(restSummary, 'transportMinDistanceToPlaced_m', calculateMinPathDistanceToObstacles(bestPath, obstacles));
            bestSummary.transportFailReason = '';
            ok = true;
        else
            bestRouteType = sprintf('start_horizontal_ok_but_overpass_failed + %s', restRouteType);
            bestSummary = restSummary;
        end
        return;
    end

    stepLen = plannerOptions.transportPathStep;
    noPlacedObstacles = isempty(obstacles);
    overpassEnabled = isfield(plannerOptions, 'enableOverpassTransport') && plannerOptions.enableOverpassTransport;
    rodEnabledForTransport = isRodObstacleEnabled(plannerOptions);

    if noPlacedObstacles || ~overpassEnabled
        path = buildLinePath(startPt, stagingPt, stepLen);
        if isPathCollisionFree(path, obstacles, clearance, plannerOptions, targetPt)
            bestPath = path;
            bestRouteType = 'direct_transport_1mm_no_placed_obstacle';
            bestSummary = summarizeMultiAxisPath(path);
            bestSummary.transportOverpassUsed = false;
            bestSummary.transportOverpassY_m = NaN;
            bestSummary.transportClearance_m = plannerOptions.transportClearance;
            bestSummary.transportMinDistanceToPlaced_m = calculateMinPathDistanceToObstacles(path, obstacles);
            bestSummary.transportFailReason = '';
            ok = true;
            return;
        end

        if ~(noPlacedObstacles && overpassEnabled && rodEnabledForTransport)
            bestRouteType = 'direct_transport_failed';
            bestSummary = emptyTransportSummary(plannerOptions, '直接运输路径与障碍物冲突');
            return;
        end
    end

    hAxis = 2;
    if isfield(plannerOptions, 'rod') && isfield(plannerOptions.rod, 'heightAxis')
        hAxis = plannerOptions.rod.heightAxis;
    end

    yCandidates = [startPt(hAxis), stagingPt(hAxis)];
    if ~isempty(obstacles)
        yCandidates(end+1) = max(obstacles(:, hAxis)) + 2 * plannerOptions.sphereR + plannerOptions.overpassSurfaceGap; %#ok<AGROW>
    end
    if isRodObstacleEnabled(plannerOptions)
        rodsForY = getPlannerRods(plannerOptions);
        if ~isempty(rodsForY)
            yCandidates(end+1) = max([rodsForY.safeY]); %#ok<AGROW>
        end
    end

    yOverBase = max(yCandidates);
    feasible = {};
    cnt = 0;

    for hi = 1:numel(plannerOptions.overpassExtraHeights)
        yOver = yOverBase + plannerOptions.overpassExtraHeights(hi);
        routeCandidates = buildOverpassTransportPathCandidates(startPt, stagingPt, yOver, stepLen, plannerOptions);

        for ri = 1:numel(routeCandidates)
            path = routeCandidates{ri}.path;
            routeName = routeCandidates{ri}.name;

            [safe, failReason, minDist] = isGlobalTransportPathSafe(path, obstacles, clearance, plannerOptions, targetPt);
            if ~safe
                continue;
            end

            cnt = cnt + 1;
            feasible{cnt}.path = path; %#ok<AGROW>
            feasible{cnt}.name = sprintf('safe_overpass_%s_y%.1fmm', routeName, yOver*1e3); %#ok<AGROW>
            feasible{cnt}.summary = summarizeMultiAxisPath(path); %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassUsed = true; %#ok<AGROW>
            feasible{cnt}.summary.transportOverpassY_m = yOver; %#ok<AGROW>
            feasible{cnt}.summary.transportClearance_m = plannerOptions.transportClearance; %#ok<AGROW>
            feasible{cnt}.summary.transportMinDistanceToPlaced_m = minDist; %#ok<AGROW>
            feasible{cnt}.summary.transportFailReason = ''; %#ok<AGROW>
        end
    end

    if isempty(feasible)
        bestRouteType = 'safe_overpass_transport_failed';
        bestSummary = emptyTransportSummary(plannerOptions, '所有上方跨越运输候选均失败');
        return;
    end

    bestIdx = 1;
    for ci = 2:numel(feasible)
        a = feasible{bestIdx}.summary;
        b = feasible{ci}.summary;
        better = false;
        if b.totalSteps < a.totalSteps
            better = true;
        elseif b.totalSteps == a.totalSteps && b.totalLength_m < a.totalLength_m
            better = true;
        elseif b.totalSteps == a.totalSteps && abs(b.totalLength_m - a.totalLength_m) < 1e-12 && ...
                b.transportMinDistanceToPlaced_m > a.transportMinDistanceToPlaced_m
            better = true;
        end
        if better
            bestIdx = ci;
        end
    end

    ok = true;
    bestPath = feasible{bestIdx}.path;
    bestRouteType = feasible{bestIdx}.name;
    bestSummary = feasible{bestIdx}.summary;
end

function val = getSummaryField(S, fieldName, defaultVal)
    if nargin < 3
        defaultVal = NaN;
    end
    if isstruct(S) && isfield(S, fieldName)
        val = S.(fieldName);
    else
        val = defaultVal;
    end
end

function summary = emptyTransportSummary(plannerOptions, failReason)
    summary = struct();
    summary.totalSteps = inf;
    summary.totalLength_m = inf;
    summary.transportOverpassUsed = false;
    summary.transportOverpassY_m = NaN;
    if isfield(plannerOptions, 'transportClearance')
        summary.transportClearance_m = plannerOptions.transportClearance;
    else
        summary.transportClearance_m = NaN;
    end
    summary.transportMinDistanceToPlaced_m = NaN;
    summary.transportFailReason = failReason;
end

function candidates = buildOverpassTransportPathCandidates(startPt, stagingPt, yOver, stepLen, plannerOptions)
    candidates = {};
    hAxis = 2;
    if isfield(plannerOptions, 'rod') && isfield(plannerOptions.rod, 'heightAxis')
        hAxis = plannerOptions.rod.heightAxis;
    end

    liftPt = startPt;
    liftPt(hAxis) = yOver;

    aboveStage = stagingPt;
    aboveStage(hAxis) = yOver;

    path1 = buildPolylinePath([startPt; liftPt; aboveStage; stagingPt], stepLen);
    candidates{end+1}.name = 'polyline'; %#ok<AGROW>
    candidates{end}.path = path1;

    offsets = plannerOptions.overpassBezierOffsets;
    if isempty(offsets)
        offsets = 0;
    end

    v = aboveStage - liftPt;
    v(hAxis) = 0;
    if norm(v) < 1e-12
        perp = [0, 0, 1];
    else

        perp = [-v(3), 0, v(1)];
        if norm(perp) < 1e-12
            perp = [0, 0, 1];
        else
            perp = perp / norm(perp);
        end
    end

    for oi = 1:numel(offsets)
        off = offsets(oi);
        if abs(off) < 1e-12
            continue;
        end
        ctrl = 0.5 * (liftPt + aboveStage) + off * perp;
        ctrl(hAxis) = yOver;
        highCurve = buildQuadraticBezierPath(liftPt, ctrl, aboveStage, stepLen);
        pStartLift = buildLinePath(startPt, liftPt, stepLen);
        pDown = buildLinePath(aboveStage, stagingPt, stepLen);
        path = concatPaths(concatPaths(pStartLift, highCurve), pDown);
        candidates{end+1}.name = sprintf('bezierOffset_%+.1fmm', off*1e3); %#ok<AGROW>
        candidates{end}.path = path;
    end
end

function path = buildPolylinePath(pts, maxStepLen)
    path = pts(1,:);
    for ii = 2:size(pts,1)
        seg = buildLinePath(pts(ii-1,:), pts(ii,:), maxStepLen);
        path = concatPaths(path, seg);
    end
end

function [ok, failReason, minDist] = isGlobalTransportPathSafe(path, obstacles, collisionClearance, plannerOptions, targetPt)
    ok = true;
    failReason = '';
    minDist = calculateMinPathDistanceToObstacles(path, obstacles);

    if isempty(obstacles)
        if ~isPathCollisionFree(path, obstacles, collisionClearance, plannerOptions, targetPt)
            ok = false;
            failReason = '运输路径与长杆等固定障碍冲突';
        end
        return;
    end

    if ~isPathCollisionFree(path, obstacles, collisionClearance, plannerOptions, targetPt)
        ok = false;
        failReason = '运输路径与已放置小球或长杆发生真实碰撞';
        return;
    end

    if segmentPathCollidesWithAnySphere(path, obstacles, plannerOptions.transportClearance)
        ok = false;
        failReason = sprintf('运输路径与已放置小球距离小于 %.1f mm', plannerOptions.transportClearance * 1e3);
        return;
    end

    if pathPassesBelowPlacedBalls(path, obstacles, plannerOptions)
        ok = false;
        failReason = '运输路径在已有小球附近低位通过，未满足上方跨越间隙';
        return;
    end
end

function tf = segmentPathCollidesWithAnySphere(path, centers, clearance)
    tf = false;
    if isempty(centers) || size(path,1) < 2
        return;
    end
    for ii = 2:size(path,1)
        if segmentCollidesWithAnySphere(path(ii-1,:), path(ii,:), centers, clearance)
            tf = true;
            return;
        end
    end
end

function tf = pathPassesBelowPlacedBalls(path, obstacles, plannerOptions)
    tf = false;
    if isempty(obstacles) || size(path,1) < 2
        return;
    end

    hAxis = 2;
    if isfield(plannerOptions, 'rod') && isfield(plannerOptions.rod, 'heightAxis')
        hAxis = plannerOptions.rod.heightAxis;
    end
    xAxis = 1;
    zAxis = 3;
    nearRadius = plannerOptions.transportClearance;
    requiredY = obstacles(:,hAxis) + 2 * plannerOptions.sphereR + plannerOptions.overpassSurfaceGap;
    sampleStep = min(0.5e-3, max(0.1e-3, plannerOptions.transportPathStep / 2));

    for ii = 2:size(path,1)
        p0 = path(ii-1,:);
        p1 = path(ii,:);
        L = norm(p1 - p0);
        n = max(1, ceil(L / sampleStep));
        for kk = 0:n
            t = kk / n;
            p = p0 + t * (p1 - p0);
            dxz = obstacles(:,[xAxis,zAxis]) - p([xAxis,zAxis]);
            horizDist = sqrt(sum(dxz.^2, 2));
            closeIdx = horizDist <= nearRadius;
            if any(closeIdx)
                if p(hAxis) < max(requiredY(closeIdx)) - 1e-12
                    tf = true;
                    return;
                end
            end
        end
    end
end

function minDist = calculateMinPathDistanceToObstacles(path, obstacles)
    if isempty(obstacles) || size(path,1) < 2
        minDist = inf;
        return;
    end
    minDist = inf;
    for ii = 2:size(path,1)
        a = path(ii-1,:);
        b = path(ii,:);
        for jj = 1:size(obstacles,1)
            d = pointToSegmentDistance(obstacles(jj,:), a, b);
            if d < minDist
                minDist = d;
            end
        end
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

        rodsForCandidates = getPlannerRods(plannerOptions);
        for rr = 1:numel(rodsForCandidates)
            thisRod = rodsForCandidates(rr);
            rodAboveTargetWpsSet = buildRodAboveTargetHorizontalEntryWaypointCandidates(startPt, targetPt, thisRod);
            for kk = 1:numel(rodAboveTargetWpsSet)
                candidates{end+1} = struct('name', sprintf('rod%d_above_target_horizontal_entry_%02d', rr, kk), ...
                    'wps', removeDuplicateWaypoints(startPt, rodAboveTargetWpsSet{kk})); %#ok<AGROW>
            end

            rodWps = buildRodSafeHorizontalCrossWaypoints(startPt, targetPt, thisRod);
            if ~isempty(rodWps)
                candidates{end+1} = struct('name', sprintf('rod%d_safe_horizontal_cross', rr), ...
                    'wps', removeDuplicateWaypoints(startPt, rodWps)); %#ok<AGROW>
            end
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

    activeRodFor2D = getActiveRodForPoint(plannerOptions, targetPt);
    if isRodObstacleEnabled(plannerOptions) && ~isempty(activeRodFor2D) && isTargetAboveRodRegion(targetPt, activeRodFor2D)
        fprintf('    规则路径失败，尝试 rod-above 2D X-Z A* 水平绕障。\n');
        drawnow;
        tmpPlannerOptions = plannerOptions;
        tmpPlannerOptions.rod = activeRodFor2D;
        [ok2D, path2D, route2D, summary2D] = planRodAboveTarget2DXZPath( ...
            startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, tmpPlannerOptions);
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

        if ~segmentSatisfiesSkeletonOutlineClearance(p0, p1, plannerOptions, targetPt)
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

function [supportRods, supportBallIDs] = buildAdaptiveSupportRodsFromTargets( ...
    targets, sphereR, rodSize, topGap, rodCount, xFractions, safetyMargin, nearTopGap, crossingGap)

    %#ok<INUSD>
    n = numel(targets);
    pts = zeros(n,3);
    sourceCurves = strings(n,1);
    gids = nan(n,1);
    for ii = 1:n
        pts(ii,:) = targets(ii).targetPoint;
        if isfield(targets, 'sourceCurves')
            sourceCurves(ii) = string(targets(ii).sourceCurves);
        end
        if isfield(targets, 'globalID')
            gids(ii) = targets(ii).globalID;
        else
            gids(ii) = ii;
        end
    end

    xmin = min(pts(:,1)); xmax = max(pts(:,1)); xRange = max(xmax-xmin, eps);
    zmin = min(pts(:,3)); zmax = max(pts(:,3)); zRange = max(zmax-zmin, eps);
    zCenter = 0.5*(zmin+zmax);

    supportBallIDs = zeros(rodCount,1);
    chosen = false(n,1);
    supportRods = repmat(makeRodTemplate(), rodCount, 1);

    supportBallIDs(1) = 1;
    chosen(1) = true;
    rod1X = pts(1,1);
    supportRods(1) = makeSupportRodFromBall(1, pts(1,:), sphereR, rodSize, topGap, ...
        safetyMargin, nearTopGap, crossingGap, gids(1), 'forced_first_ball_support');

    if rodCount <= 1
        return;
    end

    minRodXSeparation = min(6.0e-3, 0.12*xRange);

    for rr = 2:rodCount
        if rr == 2
            sideName = 'left';
            roleName = 'adaptive_left_support';
            sideMask = pts(:,1) < rod1X - minRodXSeparation;
        elseif rr == 3
            sideName = 'right';
            roleName = 'adaptive_right_support';
            sideMask = pts(:,1) > rod1X + minRodXSeparation;
        else
            sideName = 'extra';
            roleName = sprintf('adaptive_extra_support_%d', rr);
            sideMask = ~chosen;
        end

        candidateIDs = find(sideMask & ~chosen);

        if isempty(candidateIDs)
            warning('Rod %d 的 %s 侧没有满足最小 X 间距的候选点，自动放宽候选范围。', rr, sideName);
            candidateIDs = find(~chosen);
        end
        if isempty(candidateIDs)
            error('无法为第 %d 根支撑杆找到候选点。', rr);
        end

        if strcmp(sideName, 'left')
            sideXCenter = median(pts(candidateIDs,1));
            targetX = 0.5*(min(pts(candidateIDs,1)) + rod1X);
            if isfinite(sideXCenter)
                targetX = 0.5*targetX + 0.5*sideXCenter;
            end
        elseif strcmp(sideName, 'right')
            sideXCenter = median(pts(candidateIDs,1));
            targetX = 0.5*(max(pts(candidateIDs,1)) + rod1X);
            if isfinite(sideXCenter)
                targetX = 0.5*targetX + 0.5*sideXCenter;
            end
        else
            targetX = xmin + 0.5*xRange;
        end

        bestID = NaN;
        bestScore = inf;

        for kk = 1:numel(candidateIDs)
            id = candidateIDs(kk);
            xCost = abs(pts(id,1)-targetX) / xRange;
            zCost = abs(pts(id,3)-zCenter) / zRange;
            src = lower(sourceCurves(id));
            skeletonBonus = 0.0;
            if contains(src, 'skeleton')
                skeletonBonus = -0.35;
            elseif contains(src, 'outline')
                skeletonBonus = 0.08;
            end

            xNorm = (pts(id,1)-xmin) / xRange;
            edgePenalty = 0;
            if xNorm < 0.05 || xNorm > 0.95
                edgePenalty = edgePenalty + 0.4;
            end

            sepPenalty = 0;
            for jj = 1:rr-1
                prev = supportBallIDs(jj);
                if prev > 0
                    dx = abs(pts(id,1)-pts(prev,1));
                    if dx < minRodXSeparation
                        sepPenalty = sepPenalty + 5.0*(minRodXSeparation-dx)/max(minRodXSeparation, eps);
                    elseif dx < 2.0*minRodXSeparation
                        sepPenalty = sepPenalty + 1.2*(2.0*minRodXSeparation-dx)/max(minRodXSeparation, eps);
                    end
                end
            end

            sidePenalty = 0;
            if strcmp(sideName, 'left') && pts(id,1) >= rod1X
                sidePenalty = sidePenalty + 3.0;
            elseif strcmp(sideName, 'right') && pts(id,1) <= rod1X
                sidePenalty = sidePenalty + 3.0;
            end

            score = 2.2*xCost + 0.9*zCost + skeletonBonus + edgePenalty + sepPenalty + sidePenalty;
            if score < bestScore
                bestScore = score;
                bestID = id;
            end
        end

        if isnan(bestID)
            error('无法为第 %d 根支撑杆选择合适支撑球。', rr);
        end

        chosen(bestID) = true;
        supportBallIDs(rr) = bestID;
        supportRods(rr) = makeSupportRodFromBall(rr, pts(bestID,:), sphereR, rodSize, topGap, ...
            safetyMargin, nearTopGap, crossingGap, gids(bestID), roleName);
    end
end

function rod = makeRodTemplate()
    rod = struct();
    rod.center = [NaN NaN NaN];
    rod.size = [NaN NaN NaN];
    rod.heightAxis = 2;
    rod.longAxis = 3;
    rod.collisionMargin = 0;
    rod.extraHeightGap = NaN;
    rod.aboveBufferX = NaN;
    rod.aboveBufferZ = NaN;
    rod.verticalTolerance = 1e-9;
    rod.targetAboveRodMode = 'allow_lower_target';
    rod.aboveRuleMode = 'near_top_band_only';
    rod.nearTopConstantHeightGap = NaN;
    rod.crossingMinSurfaceGap = NaN;
    rod.disableFinalApproachAboveRegion = false;
    rod.allowLowSupportTarget = [];
    rod.allowLowSupportTargetTolerance = NaN;
    rod.allowLowSupportSurfaceGap = NaN;
    rod.supportBallGlobalID = NaN;
    rod.rodIndex = NaN;
    rod.rodRole = '';
    rod.zCenterFixedToZero = true;
    rod.zRange_m = [NaN NaN];
    rod.sphereR = NaN;
    rod.halfSize = [NaN NaN NaN];
    rod.minCorner = [NaN NaN NaN];
    rod.maxCorner = [NaN NaN NaN];
    rod.topY = NaN;
    rod.safeY = NaN;
    rod.collisionInflate = NaN;
    rod.inflatedMinCorner = [NaN NaN NaN];
    rod.inflatedMaxCorner = [NaN NaN NaN];
    rod.aboveMinX = NaN;
    rod.aboveMaxX = NaN;
    rod.aboveMinZ = NaN;
    rod.aboveMaxZ = NaN;
end

function rod = makeSupportRodFromBall(idx, ballCenter, sphereR, rodSize, topGap, safetyMargin, nearTopGap, crossingGap, globalID, roleName)
    if nargin < 10 || isempty(roleName)
        roleName = sprintf('support_rod_%d', idx);
    end
    rod = makeRodTemplate();
    rod.center = [ballCenter(1), ballCenter(2) - sphereR - topGap - rodSize(2)/2, 0];
    rod.size = rodSize;
    rod.heightAxis = 2;
    rod.longAxis = 3;
    rod.collisionMargin = safetyMargin;
    rod.extraHeightGap = crossingGap;
    rod.aboveBufferX = sphereR + safetyMargin;
    rod.aboveBufferZ = sphereR + safetyMargin;
    rod.verticalTolerance = 1e-9;
    rod.targetAboveRodMode = 'allow_lower_target';
    rod.aboveRuleMode = 'near_top_band_only';
    rod.nearTopConstantHeightGap = nearTopGap;
    rod.crossingMinSurfaceGap = crossingGap;
    rod.disableFinalApproachAboveRegion = false;
    rod.allowLowSupportTarget = ballCenter;
    rod.allowLowSupportTargetTolerance = 0.25e-3;
    rod.allowLowSupportSurfaceGap = topGap;
    rod.supportBallGlobalID = globalID;
    rod.rodIndex = idx;
    rod.rodRole = char(roleName);
    rod.zCenterFixedToZero = true;
    rod.zRange_m = [rod.center(3)-rod.size(3)/2, rod.center(3)+rod.size(3)/2];
end

function rods = getPlannerRods(plannerOptions)
    rods = struct([]);
    if isfield(plannerOptions, 'supportRods') && ~isempty(plannerOptions.supportRods)
        rods = plannerOptions.supportRods;
        return;
    end
    if isfield(plannerOptions, 'rod') && ~isempty(plannerOptions.rod)
        rods = plannerOptions.rod;
    end
end

function rods = getPlannerRodsForPlot(plannerOptions)
    rods = getPlannerRods(plannerOptions);
end

function rod = getActiveRodForPoint(plannerOptions, pt)
    rod = [];
    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end
    rods = getPlannerRods(plannerOptions);
    bestIdx = NaN;
    bestD = inf;
    for rr = 1:numel(rods)
        if isTargetAboveRodRegion(pt, rods(rr))
            d = hypot(pt(1)-rods(rr).center(1), pt(3)-rods(rr).center(3));
            if d < bestD
                bestD = d;
                bestIdx = rr;
            end
        end
    end
    if ~isnan(bestIdx)
        rod = rods(bestIdx);
    end
end

function val = getfieldWithDefault(s, fieldName, defaultVal)
    if isstruct(s) && isfield(s, fieldName)
        val = s.(fieldName);
    else
        val = defaultVal;
    end
end

function drawRodObstacles(rods)
    for rr = 1:numel(rods)
        drawRodObstacle(rods(rr));
        text(rods(rr).center(1), rods(rr).topY + 1.0e-3, rods(rr).center(3), ...
            sprintf(' rod %d', rr), 'FontSize', 8, 'FontWeight', 'bold');
    end
end

function rod = finalizeRodOptions(rod, sphereR)
    rod.sphereR = sphereR;
    if ~isfield(rod, 'nearTopConstantHeightGap')
        rod.nearTopConstantHeightGap = inf;
    end
    if ~isfield(rod, 'crossingMinSurfaceGap')
        rod.crossingMinSurfaceGap = rod.extraHeightGap;
    end
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
    tf = false;
    if ~isfield(plannerOptions, 'enableRodObstacle') || ~plannerOptions.enableRodObstacle
        return;
    end
    rods = getPlannerRods(plannerOptions);
    tf = ~isempty(rods);
end

function info = fillRodInfoForTarget(info, targetPt, plannerOptions)
    if isRodObstacleEnabled(plannerOptions)
        rods = getPlannerRods(plannerOptions);
        info.rodObstacleEnabled = true;
        info.rodConstraintChecked = true;
        info.rodSafeY = max([rods.safeY]);
        activeRod = getActiveRodForPoint(plannerOptions, targetPt);
        info.targetAboveRodRegion = ~isempty(activeRod);
        if ~isempty(activeRod)
            info.rodSafeY = activeRod.safeY;
        end
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
    rods = getPlannerRods(plannerOptions);
    for rr = 1:numel(rods)
        tmpOptions = plannerOptions;
        tmpOptions.rod = rods(rr);
        if ~segmentIsValidWithSingleRod(p0, p1, tmpOptions, targetPt)
            valid = false;
            return;
        end
    end
end

function valid = segmentIsValidWithSingleRod(p0, p1, plannerOptions, targetPt)
    valid = true;

    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end

    rod = plannerOptions.rod;
    hAxis = rod.heightAxis;
    sphereR = getRodSphereRadius(rod, plannerOptions);

    if segmentIntersectsAABB(p0, p1, rod.inflatedMinCorner, rod.inflatedMaxCorner)
        valid = false;
        return;
    end

    if ~segmentTouchesRodAboveRegion(p0, p1, rod)
        return;
    end

    isLowSupportTarget = isRodLowSupportTarget(targetPt, rod);
    isTargetOnThisRod  = isTargetAboveRodRegion(targetPt, rod);
    gap0 = p0(hAxis) - sphereR - rod.topY;
    gap1 = p1(hAxis) - sphereR - rod.topY;
    minSurfaceGap = min(gap0, gap1);
    requiredGap = rod.crossingMinSurfaceGap;

    supportTol = 0.35e-3;
    if isfield(rod, 'allowLowSupportTargetTolerance') && rod.allowLowSupportTargetTolerance > 0
        supportTol = max(supportTol, rod.allowLowSupportTargetTolerance);
    end
    rodTargetTol = max(8.0e-3, supportTol);

    isSupportFinalSegment = isLowSupportTarget && segmentEndpointNearTarget(p0, p1, targetPt, supportTol);
    isRodTargetFinalSegment = isTargetOnThisRod && segmentEndpointNearTarget(p0, p1, targetPt, rodTargetTol);
    allowedLowGap = 0.20e-3;
    if isfield(rod, 'allowLowSupportSurfaceGap') && rod.allowLowSupportSurfaceGap > 0
        allowedLowGap = min(allowedLowGap, rod.allowLowSupportSurfaceGap);
    end
    if isTargetOnThisRod && all(isfinite(targetPt))
        targetGap = targetPt(hAxis) - sphereR - rod.topY;
        allowedLowGap = max(0.20e-3, min(allowedLowGap, targetGap + 1e-9));
    end

    if minSurfaceGap < requiredGap - 1e-12
        if ~((isSupportFinalSegment || isRodTargetFinalSegment) && minSurfaceGap >= allowedLowGap - 1e-12)
            valid = false;
            return;
        end
    end

    if minSurfaceGap <= rod.nearTopConstantHeightGap + 1e-12
        dy = abs(p1(hAxis) - p0(hAxis));
        if dy > rod.verticalTolerance
            if ~(isSupportFinalSegment || isRodTargetFinalSegment)
                valid = false;
                return;
            end
        end
    end
end

function sphereR = getRodSphereRadius(rod, plannerOptions)
    if isfield(rod, 'sphereR') && rod.sphereR > 0
        sphereR = rod.sphereR;
    elseif isfield(plannerOptions, 'sphereR') && plannerOptions.sphereR > 0
        sphereR = plannerOptions.sphereR;
    else
        sphereR = rod.collisionInflate - rod.collisionMargin;
    end
end

function tf = isRodLowSupportTarget(targetPt, rod)
    tf = false;
    if nargin < 2 || ~isfield(rod, 'allowLowSupportTarget') || isempty(rod.allowLowSupportTarget)
        return;
    end
    if nargin < 1 || any(~isfinite(targetPt))
        return;
    end
    tol = 0.25e-3;
    if isfield(rod, 'allowLowSupportTargetTolerance') && rod.allowLowSupportTargetTolerance > 0
        tol = rod.allowLowSupportTargetTolerance;
    end
    tf = norm(targetPt - rod.allowLowSupportTarget) <= tol;
end

function tf = segmentCrossesRodShortAxis(p0, p1, rod)
    if ~segmentTouchesRodAboveRegion(p0, p1, rod)
        tf = false;
        return;
    end
    leftSide0  = p0(1) < rod.aboveMinX - 1e-12;
    rightSide0 = p0(1) > rod.aboveMaxX + 1e-12;
    leftSide1  = p1(1) < rod.aboveMinX - 1e-12;
    rightSide1 = p1(1) > rod.aboveMaxX + 1e-12;
    tf = (leftSide0 && rightSide1) || (rightSide0 && leftSide1);
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
    tf = ~isempty(getActiveRodForPoint(plannerOptions, targetPt));
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

function generateSpiralBallCsv(outCsv)
    ballDiameter_mm = 2.20;
    nBalls = 60;
    turns = 4.0;
    baseRadius_mm = 9.0;
    topRadius_mm  = 1.2;
    height_mm = 36.0;
    startAngle_deg = 0;
    denseSampleN = 20000;
    theta0 = deg2rad(startAngle_deg);
    thetaEnd = theta0 + 2*pi*turns;
    thetaDense = linspace(theta0, thetaEnd, denseSampleN).';
    sDense = (thetaDense - theta0) / (thetaEnd - theta0);
    rDense = baseRadius_mm + (topRadius_mm - baseRadius_mm) .* sDense;
    yDense = height_mm .* sDense;
    xDense = rDense .* cos(thetaDense);
    zDense = rDense .* sin(thetaDense);
    densePts = [xDense, yDense, zDense];

    densePts(:,2) = densePts(:,2) - min(densePts(:,2));

    [ballPts, arcLength_mm] = resampleCurveByArcLength_local(densePts, nBalls);

    adjDist = sqrt(sum(diff(ballPts, 1, 1).^2, 2));
    minAdjDist = min(adjDist);
    maxAdjDist = max(adjDist);
    minDistAll = minPairwiseDistance_local(ballPts);

    fprintf('已自动生成螺旋上升小球 CSV。\n');
    fprintf('小球数量：%d；小球直径：%.3f mm；螺旋圈数：%.2f。\n', nBalls, ballDiameter_mm, turns);
    fprintf('底部半径 %.3f mm，顶部半径 %.3f mm，总高度 %.3f mm。\n', baseRadius_mm, topRadius_mm, height_mm);
    fprintf('中心线弧长 %.3f mm；相邻球心距 min %.3f mm, max %.3f mm, mean %.3f mm。\n', ...
        arcLength_mm, minAdjDist, maxAdjDist, mean(adjDist));
    fprintf('全局最小球心距 %.3f mm。\n', minDistAll);

    if minDistAll < ballDiameter_mm - 1e-9
        warning('螺旋点存在重叠风险：全局最小球心距 %.3f mm < 小球直径 %.3f mm。', ...
            minDistAll, ballDiameter_mm);
    end

    rows = cell(nBalls, 9);
    for i = 1:nBalls
        globalID = i;
        zLayerID = i;
        orderInLayer = i;
        theta_i = theta0 + (thetaEnd - theta0) * (i - 1) / max(nBalls - 1, 1);
        thetaDeg = rad2deg(theta_i);
        sourceClass = 'spiral';
        sourceCurves = 'ascending_spiral';
        x_mm = ballPts(i,1);
        y_mm = ballPts(i,2);
        z_mm = ballPts(i,3);

        rows(i,:) = {globalID, zLayerID, orderInLayer, thetaDeg, ...
            sourceClass, sourceCurves, x_mm, y_mm, z_mm};
    end

    T = cell2table(rows, 'VariableNames', { ...
        'global_id', 'z_layer_id', 'order_in_layer', 'theta_deg', ...
        'source_class', 'source_curves', 'x_mm', 'y_mm', 'z_mm'});

    writetable(T, outCsv);
    fprintf('螺旋上升件 CSV 已写入：%s\n', outCsv);
end

function [resampledPts, totalLength] = resampleCurveByArcLength_local(densePts, nPts)
    if size(densePts, 1) < 2
        error('densePts 至少需要两个点。');
    end

    seg = sqrt(sum(diff(densePts, 1, 1).^2, 2));
    cumLen = [0; cumsum(seg)];
    totalLength = cumLen(end);

    if totalLength <= 0
        error('曲线总长度为 0，无法重采样。');
    end

    targetLen = linspace(0, totalLength, nPts).';
    resampledPts = zeros(nPts, 3);
    for dim = 1:3
        resampledPts(:,dim) = interp1(cumLen, densePts(:,dim), targetLen, 'linear');
    end
end

function md = minPairwiseDistance_local(pts)
    n = size(pts,1);
    md = inf;
    if n <= 1
        return;
    end

    for ii = 1:n-1
        d = sqrt(sum((pts(ii+1:end,:) - pts(ii,:)).^2, 2));
        localMin = min(d);
        if localMin < md
            md = localMin;
        end
    end
end

function plannerOptionsOut = preparePlannerOptionsForCurrentTarget( ...
    plannerOptionsIn, targets, currentIdx, placedCenters, placedPointIndices, placedGlobalIDs)

    plannerOptionsOut = plannerOptionsIn;
    plannerOptionsOut.currentPointIndex = targets(currentIdx).pointIndex;
    plannerOptionsOut.currentGlobalID = targets(currentIdx).globalID;
    plannerOptionsOut.currentSourceCurves = char(targets(currentIdx).sourceCurves);
    plannerOptionsOut.currentSourceClass = char(targets(currentIdx).sourceClass);
    plannerOptionsOut.currentTargetIndex = currentIdx;
    plannerOptionsOut.currentIsSkeleton = isTextSkeleton(plannerOptionsOut.currentSourceCurves) || ...
        isTextSkeleton(plannerOptionsOut.currentSourceClass);
    plannerOptionsOut.currentIsOutline = isTextOutline(plannerOptionsOut.currentSourceCurves) || ...
        isTextOutline(plannerOptionsOut.currentSourceClass);

    plannerOptionsOut.placedSourceCurves = {};
    plannerOptionsOut.placedSourceClass = {};
    plannerOptionsOut.placedOutlineCenters = zeros(0,3);
    plannerOptionsOut.placedOutlinePointIndices = [];
    plannerOptionsOut.placedOutlineGlobalIDs = [];
    plannerOptionsOut.placedCenters = placedCenters;
    plannerOptionsOut.placedPointIndices = placedPointIndices;
    plannerOptionsOut.placedGlobalIDs = placedGlobalIDs;

    if isempty(placedPointIndices)
        return;
    end

    outlineMask = false(numel(placedPointIndices),1);
    for kk = 1:numel(placedPointIndices)
        idx = placedPointIndices(kk);
        if idx >= 1 && idx <= numel(targets)
            src = char(targets(idx).sourceCurves);
            cls = char(targets(idx).sourceClass);
            plannerOptionsOut.placedSourceCurves{kk,1} = src;
            plannerOptionsOut.placedSourceClass{kk,1} = cls;
            outlineMask(kk) = isTextOutline(src) || isTextOutline(cls);
        else
            plannerOptionsOut.placedSourceCurves{kk,1} = '';
            plannerOptionsOut.placedSourceClass{kk,1} = '';
        end
    end

    plannerOptionsOut.placedOutlineCenters = placedCenters(outlineMask,:);
    plannerOptionsOut.placedOutlinePointIndices = placedPointIndices(outlineMask);
    plannerOptionsOut.placedOutlineGlobalIDs = placedGlobalIDs(outlineMask);
end

function tf = isTextSkeleton(txt)
    s = lower(string(txt));
    tf = contains(s, "skeleton") || contains(s, "vein") || contains(s, "branch");
end

function tf = isTextOutline(txt)
    s = lower(string(txt));
    tf = contains(s, "outline");
end

function tf = isCurrentSkeletonBall(plannerOptions)
    tf = false;
    if isfield(plannerOptions, 'currentIsSkeleton')
        tf = plannerOptions.currentIsSkeleton;
        return;
    end
    if isfield(plannerOptions, 'currentSourceCurves')
        tf = isTextSkeleton(plannerOptions.currentSourceCurves);
    end
end

function tf = segmentSatisfiesSkeletonOutlineClearance(p0, p1, plannerOptions, targetPt)
    tf = true;
    if ~isfield(plannerOptions, 'enableSkeletonOutlineClearance') || ~plannerOptions.enableSkeletonOutlineClearance
        return;
    end
    if ~isCurrentSkeletonBall(plannerOptions)
        return;
    end
    if ~isfield(plannerOptions, 'placedOutlineCenters') || isempty(plannerOptions.placedOutlineCenters)
        return;
    end

    required = 3.0e-3;
    if isfield(plannerOptions, 'skeletonToOutlineClearance') && plannerOptions.skeletonToOutlineClearance > 0
        required = plannerOptions.skeletonToOutlineClearance;
    end

    clearance = 2.2e-3;
    if isfield(plannerOptions, 'sphereR') && plannerOptions.sphereR > 0
        clearance = 2 * plannerOptions.sphereR;
    end
    contactTol = 0.35e-3;
    if isfield(plannerOptions, 'contactTolerance') && plannerOptions.contactTolerance > 0
        contactTol = plannerOptions.contactTolerance;
    end

    centers = plannerOptions.placedOutlineCenters;
    exempt = sqrt(sum((centers - targetPt).^2, 2)) <= clearance + contactTol + 0.15e-3;

    for ii = 1:size(centers,1)
        if exempt(ii)
            continue;
        end
        d = pointToSegmentDistance(centers(ii,:), p0, p1);
        if d < required - 1e-12
            tf = false;
            return;
        end
    end
end

function [ok, bestPath, bestRouteType, bestSummary, info] = planSkeletonCurvedFinalApproach( ...
    startPt, targetPt, obstacles, clearance, allowedSteps, gridBase, ...
    plannerOptions, placedPointIndices, placedGlobalIDs, info)

    ok = false;
    bestPath = [];
    bestRouteType = '';
    bestSummary = [];

    if ~isCurrentSkeletonBall(plannerOptions)
        return;
    end
    if isempty(obstacles)
        return;
    end

    parentCandidates = findApproachParentCandidates( ...
        targetPt, obstacles, placedPointIndices, placedGlobalIDs, clearance, plannerOptions);
    if isempty(parentCandidates)
        info.skeletonOutlineFailReason = '内部骨架曲线贴近失败：未找到接触父球';
        return;
    end

    if isfield(plannerOptions, 'finalApproachDistanceCandidates') && ~isempty(plannerOptions.finalApproachDistanceCandidates)
        distCandidates = plannerOptions.finalApproachDistanceCandidates(:)';
    else
        distCandidates = plannerOptions.finalApproachDistance;
    end

    feasible = {};
    cnt = 0;
    lastFail = '';

    for pi = 1:numel(parentCandidates)
        parent = parentCandidates(pi);
        dirVec = targetPt - parent.center;
        if norm(dirVec) <= 1e-12
            continue;
        end
        u = dirVec / norm(dirVec);

        u(2) = 0;
        if norm(u) <= 1e-12
            u = (targetPt - parent.center) / norm(targetPt - parent.center);
        else
            u = u / norm(u);
        end

        for di = 1:numel(distCandidates)
            dApproach = distCandidates(di);
            entryPoint = targetPt + dApproach * u;
            entryPoint(2) = targetPt(2);

            [okEntryCtrl, entryGap, entryRodIdx, entryReason] = checkRodControlPointClearance(entryPoint, plannerOptions);
            if ~okEntryCtrl
                lastFail = sprintf('曲线入口点离支撑杆过近：rod %d, surfaceGap %.3f mm, %s', ...
                    entryRodIdx, entryGap*1e3, entryReason);
                continue;
            end

            curveCandidates = buildSkeletonCurvedFinalCandidates(entryPoint, targetPt, parent.center, plannerOptions);
            for ci = 1:numel(curveCandidates)
                finalPath = curveCandidates{ci}.path;

                if isfield(curveCandidates{ci}, 'controlPoint')
                    [okCurveCtrl, curveCtrlGap, curveCtrlRodIdx, curveCtrlReason] = checkRodControlPointClearance(curveCandidates{ci}.controlPoint, plannerOptions);
                    if ~okCurveCtrl
                        lastFail = sprintf('%s 控制点离支撑杆过近：rod %d, surfaceGap %.3f mm, %s', ...
                            curveCandidates{ci}.type, curveCtrlRodIdx, curveCtrlGap*1e3, curveCtrlReason);
                        continue;
                    end
                else
                    curveCtrlGap = entryGap;
                    curveCtrlRodIdx = entryRodIdx;
                end

                if ~isPathCollisionFree(finalPath, obstacles, clearance, plannerOptions, targetPt)
                    lastFail = sprintf('%s final curve failed collision/outline clearance', curveCandidates{ci}.type);
                    continue;
                end

                [okSkelFinal, skelMinD, skelNearGID, skelReason] = validateSkeletonFinalApproachOutlineClearance(finalPath, plannerOptions, targetPt);
                if ~okSkelFinal
                    lastFail = sprintf('%s final curve 外轮廓距离不足：nearest outline globalID %.0f, minDist %.3f mm, %s', ...
                        curveCandidates{ci}.type, skelNearGID, skelMinD*1e3, skelReason);
                    continue;
                end

                [okGlobal, globalPath, globalRouteType, globalSummary] = planAdaptiveTransportToStaging( ...
                    startPt, finalPath(1,:), obstacles, clearance, allowedSteps, gridBase, plannerOptions, targetPt);
                if ~okGlobal
                    lastFail = sprintf('%s final curve feasible but transport to entry failed', curveCandidates{ci}.type);
                    continue;
                end

                path = concatPaths(globalPath, finalPath);
                summary = summarizeMultiAxisPath(path);
                summary.globalRouteType = globalRouteType;
                summary.finalApproachRouteType = sprintf('skeleton_%s_final_approach_d%.1fmm', curveCandidates{ci}.type, dApproach*1e3);
                summary.transportOverpassUsed = getSummaryField(globalSummary, 'transportOverpassUsed', false);
                summary.transportOverpassY_m = getSummaryField(globalSummary, 'transportOverpassY_m', NaN);
                summary.transportClearance_m = getSummaryField(globalSummary, 'transportClearance_m', plannerOptions.transportClearance);
                summary.transportMinDistanceToPlaced_m = getSummaryField(globalSummary, 'transportMinDistanceToPlaced_m', calculateMinPathDistanceToObstacles(globalPath, obstacles));
                summary.transportFailReason = '';
                candInfo = info;
                candInfo.parentIndex = parent.index;
                candInfo.parentGlobalID = parent.globalID;
                candInfo.parentDistance = parent.distance;
                candInfo.approachDirection = u;
                candInfo.preApproachPoint = finalPath(1,:);
                candInfo.finalApproachRouteType = summary.finalApproachRouteType;
                candInfo.finalApproachUsed = true;
                candInfo.finalApproachPath = finalPath;
                candInfo.globalRouteType = globalRouteType;
                candInfo.failReason = '';
                candInfo.isSkeletonBall = true;
                candInfo.skeletonOutlineClearanceChecked = true;
                candInfo.skeletonOutlineClearance_m = getfieldWithDefault(plannerOptions, 'skeletonToOutlineClearance', 3.0e-3);
                candInfo.skeletonOutlineFinalApproachUsed = true;
                candInfo.skeletonOutlineFinalApproachType = curveCandidates{ci}.type;
                candInfo.skeletonOutlineFailReason = '';
                candInfo.skeletonFinalOutlineMinDistance_m = skelMinD;
                candInfo.skeletonFinalOutlineNearestGlobalID = skelNearGID;
                candInfo.rodControlPointClearanceChecked = true;
                candInfo.rodControlPointMinGap_m = min(entryGap, curveCtrlGap);
                candInfo.rodControlPointNearestRodID = curveCtrlRodIdx;
                candInfo.rodControlPointFailReason = '';
                candInfo = fillTransportInfoFromSummary(candInfo, globalRouteType, globalSummary, plannerOptions);
                candInfo = updateStartHorizontalInfoFromRoute(candInfo, globalRouteType, plannerOptions);
                cnt = cnt + 1;
                feasible{cnt}.path = path; %#ok<AGROW>
                feasible{cnt}.routeType = sprintf('%s + %s', globalRouteType, summary.finalApproachRouteType); %#ok<AGROW>
                feasible{cnt}.summary = summary; %#ok<AGROW>
                feasible{cnt}.info = candInfo; %#ok<AGROW>
                feasible{cnt}.approachDistance = dApproach; %#ok<AGROW>
                feasible{cnt}.routePriority = getTransportRoutePriority(globalRouteType); %#ok<AGROW>
                feasible{cnt}.parentDist = parent.distance; %#ok<AGROW>
            end
        end
    end

    if isempty(feasible)
        info.skeletonOutlineFailReason = sprintf('内部骨架Bezier/圆弧末端贴近均失败。%s', lastFail);
        info.failReason = info.skeletonOutlineFailReason;
        return;
    end

    bestIdx = 1;
    bestScore = finalApproachCandidateScore(feasible{1}, plannerOptions);
    for ci = 2:numel(feasible)
        thisScore = finalApproachCandidateScore(feasible{ci}, plannerOptions);
        if lexicographicScoreBetter(thisScore, bestScore)
            bestIdx = ci;
            bestScore = thisScore;
        end
    end

    chosen = feasible{bestIdx};
    ok = true;
    bestPath = chosen.path;
    bestRouteType = chosen.routeType;
    bestSummary = chosen.summary;
    info = chosen.info;
end

function candidates = buildSkeletonCurvedFinalCandidates(entryPoint, targetPt, parentCenter, plannerOptions)
    candidates = {};
    cnt = 0;
    step = 0.1e-3;
    if isfield(plannerOptions, 'finalApproachStep') && plannerOptions.finalApproachStep > 0
        step = plannerOptions.finalApproachStep;
    end
    offsets = [1.0, 1.5, 2.0, 2.5, 3.0] * 1e-3;
    if isfield(plannerOptions, 'skeletonBezierFinalOffsets') && ~isempty(plannerOptions.skeletonBezierFinalOffsets)
        offsets = plannerOptions.skeletonBezierFinalOffsets;
    end
    radii = [2.0, 3.0, 4.0, 5.0] * 1e-3;
    if isfield(plannerOptions, 'skeletonArcFinalRadii') && ~isempty(plannerOptions.skeletonArcFinalRadii)
        radii = plannerOptions.skeletonArcFinalRadii;
    end

    midPoint = 0.5 * (entryPoint + targetPt);
    awayOutline = getAwayDirectionFromNearestOutline(midPoint, targetPt, plannerOptions);
    awayRod = getAwayDirectionFromNearestRod(midPoint, plannerOptions);
    rodWeights = 1.0;
    if isfield(plannerOptions, 'skeletonBezierRodWeights') && ~isempty(plannerOptions.skeletonBezierRodWeights)
        rodWeights = plannerOptions.skeletonBezierRodWeights(1);
    end
    away = awayOutline + rodWeights * awayRod;
    away(2) = 0;
    if norm(away) < 1e-12
        away = awayOutline;
    end
    away = away / norm(away);

    approachDir = targetPt - parentCenter;
    approachDir(2) = 0;
    if norm(approachDir) < 1e-12
        approachDir = targetPt - entryPoint;
        approachDir(2) = 0;
    end
    if norm(approachDir) < 1e-12
        approachDir = [1, 0, 0];
    else
        approachDir = approachDir / norm(approachDir);
    end
    perp = [-approachDir(3), 0, approachDir(1)];
    if norm(perp) < 1e-12
        perp = away;
    end
    perp = perp / norm(perp);

    if isfield(plannerOptions, 'enableSkeletonBezierFinalApproach') && plannerOptions.enableSkeletonBezierFinalApproach
        for oi = 1:numel(offsets)
            for sg = [-1, 1]
                ctrl = midPoint + offsets(oi) * (away + 0.35*sg*perp);
                ctrl(2) = targetPt(2);
                path = buildQuadraticBezierPath(entryPoint, ctrl, targetPt, step);
                cnt = cnt + 1;
                candidates{cnt}.path = path; %#ok<AGROW>
                candidates{cnt}.controlPoint = ctrl; %#ok<AGROW>
                candidates{cnt}.type = sprintf('bezier_offset%.1fmm', offsets(oi)*1e3); %#ok<AGROW>
            end
        end
    end

    if isfield(plannerOptions, 'enableSkeletonArcFinalApproach') && plannerOptions.enableSkeletonArcFinalApproach
        for ri = 1:numel(radii)
            for sg = [-1, 1]
                ctrl = midPoint + radii(ri) * (0.65*away + sg*perp);
                ctrl(2) = targetPt(2);
                path = buildQuadraticBezierPath(entryPoint, ctrl, targetPt, step);
                cnt = cnt + 1;
                candidates{cnt}.path = path; %#ok<AGROW>
                candidates{cnt}.controlPoint = ctrl; %#ok<AGROW>
                candidates{cnt}.type = sprintf('arc_like_r%.1fmm', radii(ri)*1e3); %#ok<AGROW>
            end
        end
    end
end

function away = getAwayDirectionFromNearestOutline(queryPoint, targetPt, plannerOptions)
    away = [1, 0, 0];
    if ~isfield(plannerOptions, 'placedOutlineCenters') || isempty(plannerOptions.placedOutlineCenters)
        return;
    end
    centers = plannerOptions.placedOutlineCenters;
    clearance = 2.2e-3;
    if isfield(plannerOptions, 'sphereR') && plannerOptions.sphereR > 0
        clearance = 2 * plannerOptions.sphereR;
    end
    tol = 0.35e-3;
    if isfield(plannerOptions, 'contactTolerance') && plannerOptions.contactTolerance > 0
        tol = plannerOptions.contactTolerance;
    end
    exempt = sqrt(sum((centers - targetPt).^2, 2)) <= clearance + tol + 0.15e-3;
    usable = find(~exempt);
    if isempty(usable)
        usable = (1:size(centers,1))';
    end
    d = sqrt(sum((centers(usable,:) - queryPoint).^2, 2));
    [~, idx] = min(d);
    nearest = centers(usable(idx),:);
    away = queryPoint - nearest;
    away(2) = 0;
    if norm(away) < 1e-12
        away = [1, 0, 0];
    else
        away = away / norm(away);
    end
end

function [ok, minDist, nearGlobalID, reason] = validateSkeletonFinalApproachOutlineClearance(finalPath, plannerOptions, targetPt)
    ok = true;
    reason = '';
    minDist = inf;
    nearGlobalID = NaN;

    if ~isfield(plannerOptions, 'enableSkeletonOutlineClearance') || ~plannerOptions.enableSkeletonOutlineClearance
        return;
    end
    if ~isCurrentSkeletonBall(plannerOptions)
        return;
    end
    if ~isfield(plannerOptions, 'placedOutlineCenters') || isempty(plannerOptions.placedOutlineCenters)
        return;
    end
    if isempty(finalPath)
        return;
    end

    required = getfieldWithDefault(plannerOptions, 'skeletonToOutlineClearance', 3.0e-3);
    clearance = 2 * getfieldWithDefault(plannerOptions, 'sphereR', 1.10e-3);
    contactTol = getfieldWithDefault(plannerOptions, 'contactTolerance', 0.35e-3);
    centers = plannerOptions.placedOutlineCenters;
    gids = plannerOptions.placedOutlineGlobalIDs;
    if isempty(gids)
        gids = (1:size(centers,1))';
    end

    exempt = sqrt(sum((centers - targetPt).^2, 2)) <= clearance + contactTol + 0.15e-3;

    for ii = 1:size(centers,1)
        if exempt(ii)
            continue;
        end
        dif = finalPath - centers(ii,:);
        d = sqrt(sum(dif.^2, 2));
        localMin = min(d);
        if localMin < minDist
            minDist = localMin;
            nearGlobalID = gids(ii);
        end
        if localMin < required - 1e-12
            ok = false;
            reason = sprintf('finalApproachPath 到非豁免外轮廓球过近，要求 %.3f mm，实际 %.3f mm', required*1e3, localMin*1e3);
            return;
        end
    end
end

function [ok, minSurfaceGap, nearestRodID, reason] = checkRodControlPointClearance(pt, plannerOptions)
    ok = true;
    reason = '';
    minSurfaceGap = inf;
    nearestRodID = NaN;

    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end

    required = getfieldWithDefault(plannerOptions, 'rodControlPointMinSurfaceGap', 3.0e-3);
    sphereR = getfieldWithDefault(plannerOptions, 'sphereR', 1.10e-3);
    rods = getPlannerRods(plannerOptions);
    if isempty(rods)
        return;
    end

    for rr = 1:numel(rods)
        rod = rods(rr);
        hAxis = rod.heightAxis;
        if isTargetAboveRodRegion(pt, rod)
            gap = pt(hAxis) - sphereR - rod.topY;
        else
            dBox = pointToAABBDistance(pt, rod.minCorner, rod.maxCorner);
            gap = dBox - sphereR;
        end

        if gap < minSurfaceGap
            minSurfaceGap = gap;
            nearestRodID = rod.rodIndex;
        end
    end

    if minSurfaceGap < required - 1e-12
        ok = false;
        reason = sprintf('控制点到支撑杆表面距离不足，要求 %.3f mm，实际 %.3f mm', required*1e3, minSurfaceGap*1e3);
    end
end

function d = pointToAABBDistance(p, boxMin, boxMax)
    dx = max([boxMin(1)-p(1), 0, p(1)-boxMax(1)]);
    dy = max([boxMin(2)-p(2), 0, p(2)-boxMax(2)]);
    dz = max([boxMin(3)-p(3), 0, p(3)-boxMax(3)]);
    d = sqrt(dx*dx + dy*dy + dz*dz);
end

function away = getAwayDirectionFromNearestRod(queryPoint, plannerOptions)
    away = [0, 0, 0];
    if ~isRodObstacleEnabled(plannerOptions)
        return;
    end
    rods = getPlannerRods(plannerOptions);
    if isempty(rods)
        return;
    end

    bestGap = inf;
    bestDir = [0, 0, 0];
    for rr = 1:numel(rods)
        rod = rods(rr);
        q = queryPoint;
        closest = min(max(q, rod.minCorner), rod.maxCorner);
        dir = q - closest;
        dir(2) = 0;
        gap = norm(dir);
        if gap < bestGap
            bestGap = gap;
            bestDir = dir;
        end
    end

    if norm(bestDir) > 1e-12
        away = bestDir / norm(bestDir);
    else
        away = [1, 0, 0];
    end
end

function tf = segmentEndpointNearTarget(p0, p1, targetPt, tol)
    if nargin < 4 || isempty(tol)
        tol = 0.35e-3;
    end
    tf = norm(p0 - targetPt) <= tol || norm(p1 - targetPt) <= tol;
end
