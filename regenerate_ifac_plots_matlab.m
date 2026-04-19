function results = regenerate_ifac_plots_matlab(dataSource, outDir, varargin)
%REGENERATE_IFAC_PLOTS_MATLAB Rebuild Figures 2-5 from the IFAC paper data.
%
% MATLAB translation of regenerate_ifac_plots.py.
%
% Usage
%   results = regenerate_ifac_plots_matlab('/path/to/Data.zip');
%   results = regenerate_ifac_plots_matlab('/path/to/Data', 'out_dir');
%   results = regenerate_ifac_plots_matlab(..., 'Panel4bMode', 'sigma', ...
%                                          'HorizonSteps', 0, ...
%                                          'DPI', 200);
%
% Inputs
%   dataSource : path to Data.zip or to an extracted directory containing
%                sealed/ and coupled/.
%   outDir     : output directory for regenerated figures and summaries.
%
% Name-value options
%   Panel4bMode  : 'sigma' (default) or 'corr'
%   HorizonSteps : nonnegative integer target shift in samples (default 0)
%   DPI          : PNG export DPI (default 200)
%
% Outputs
%   results struct with trials, metrics, and Figure 5 summary fields.
%
% Notes
%   - This version follows the uploaded Python plotting script rather than
%     the earlier approximate MATLAB pipeline.
%   - It does not resample to 100 Hz. It drops non-increasing timestamps,
%     trims the first and last 10 s, and uses the raw sampling interval.
%   - Figure 2 representative traces use the common raw-sensor estimator,
%     then lightly smooth the predicted trace only, matching the Python code.
%   - Figure 5 labels the sealed topology as 'Sealed', again matching the
%     final manuscript plotting script.

if nargin < 1 || isempty(dataSource)
    if exist('Data.zip', 'file') == 2
        dataSource = 'Data.zip';
    else
        dataSource = pwd;
    end
end
if nargin < 2 || isempty(outDir)
    outDir = fullfile(pwd, 'regenerated_plots_matlab');
end

parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'Panel4bMode', 'sigma', @(x) ischar(x) || isstring(x));
addParameter(parser, 'HorizonSteps', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(parser, 'DPI', 200, @(x) isnumeric(x) && isscalar(x) && x > 0);
parse(parser, varargin{:});

opts = defaultOptions();
opts.panel4bMode = lower(char(parser.Results.Panel4bMode));
opts.horizonSteps = double(parser.Results.HorizonSteps);
opts.dpi = double(parser.Results.DPI);

if ~ismember(opts.panel4bMode, {'sigma', 'corr'})
    error('Panel4bMode must be ''sigma'' or ''corr''.');
end

[dataRoot, cleanupObj] = maybeExtractData(char(dataSource)); %#ok<NASGU>
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

trials = loadAllTrials(dataRoot, opts);
metrics = buildTrialTable(trials, opts, opts.horizonSteps);
fig5Summary = struct();

makeFig2(trials, metrics, outDir, opts);
makeFig3(trials, outDir, opts);
makeFig4(metrics, outDir, opts);
fig5Summary = makeFig5(trials, outDir, opts);
writeSummaryFiles(metrics, fig5Summary, outDir, opts);

results = struct();
results.trials = trials;
results.metrics = metrics;
results.fig5Summary = fig5Summary;
results.options = opts;
save(fullfile(outDir, 'regenerated_ifac_results.mat'), 'results', '-v7.3');

end

function opts = defaultOptions()
opts = struct();
opts.requiredCols = { ...
    'time', ...
    'Desired_pressure_segment_2', ...
    'Desired_pressure_segment_3', ...
    'Desired_pressure_segment_4', ...
    'Measured_pressure_Segment_1_pouch_1', ...
    'Measured_pressure_Segment_1_pouch_2', ...
    'Measured_pressure_Segment_1_pouch_3', ...
    'Measured_pressure_Segment_1_pouch_4', ...
    'Measured_pressure_Segment_1_pouch_5', ...
    'Rigid_body_1_qx', 'Rigid_body_1_qy', 'Rigid_body_1_qz', 'Rigid_body_1_qw', ...
    'Rigid_body_3_qx', 'Rigid_body_3_qy', 'Rigid_body_3_qz', 'Rigid_body_3_qw'};
opts.delay = 20;
opts.alpha = 0.01;
opts.trimSec = 10.0;
opts.trainFraction = 0.70;
opts.memoryK = 40;
opts.maxXcorrLagSec = 15.0;
opts.totalPouches = 5;
opts.fig2TraceSeconds = 20.0;
opts.panel4bMode = 'sigma';
opts.horizonSteps = 0;
opts.dpi = 200;
opts.sealedLabel = 'Sealed';
opts.lineWidth = 1.4;
opts.markerSize = 6;
opts.boxWidth = 0.6;
opts.boxMedianColor = [0.85, 0.55, 0.00];
opts.grayColor = [0.5, 0.5, 0.5];
opts.palette = [ ...
    hex2rgb('0072B2'); ...
    hex2rgb('E69F00'); ...
    hex2rgb('009E73'); ...
    hex2rgb('CC79A7'); ...
    hex2rgb('D55E00')];
opts.topologyColors = struct('coupled', hex2rgb('0072B2'), 'sealed', hex2rgb('E69F00'));
opts.waveformMarkers = struct('axial', 'o', 'circular', 's', 'triangular', '^');
end

function [dataRoot, cleanupObj] = maybeExtractData(dataSource)
cleanupObj = [];
if exist(dataSource, 'dir') == 7
    dataRoot = findDataRoot(dataSource);
    return;
end

if exist(dataSource, 'file') ~= 2 || ~endsWithLower(dataSource, '.zip')
    error('Expected dataSource to be a directory or a .zip file.');
end

tmpRoot = tempname;
mkdir(tmpRoot);
cleanupObj = onCleanup(@() safeRemoveDir(tmpRoot));
unzip(dataSource, tmpRoot);
dataRoot = findDataRoot(tmpRoot);
end

function dataRoot = findDataRoot(rootDir)
if isDataRoot(rootDir)
    dataRoot = rootDir;
    return;
end
listing = dir(rootDir);
for k = 1:numel(listing)
    if listing(k).isdir && ~strcmp(listing(k).name, '.') && ~strcmp(listing(k).name, '..')
        cand = fullfile(rootDir, listing(k).name);
        if isDataRoot(cand)
            dataRoot = cand;
            return;
        end
    end
end
error('Could not find parallel/ and coupled/ under %s.', rootDir);
end

function tf = isDataRoot(rootDir)
tf = exist(fullfile(rootDir, 'parallel'), 'dir') == 7 && ...
     exist(fullfile(rootDir, 'coupled'), 'dir') == 7;
end

function safeRemoveDir(pathStr)
if exist(pathStr, 'dir') == 7
    try
        rmdir(pathStr, 's');
    catch
    end
end
end

function tf = endsWithLower(strIn, suffix)
strIn = lower(char(strIn));
suffix = lower(char(suffix));
if numel(strIn) < numel(suffix)
    tf = false;
else
    tf = strcmp(strIn(end-numel(suffix)+1:end), suffix);
end
end

function trials = loadAllTrials(dataRoot, opts)
trialCell = {};
for topoCell = {'parallel', 'coupled'}
    topo = topoCell{1};
    files = dir(fullfile(dataRoot, topo, '*.csv'));
    [~, order] = sort({files.name});
    files = files(order);
    for k = 1:numel(files)
        pathStr = fullfile(files(k).folder, files(k).name);
        meta = parseTrialName(pathStr);
        trialCell{end+1} = loadTrial(meta, opts); %#ok<AGROW>
    end
end
if isempty(trialCell)
    error('No CSV files were found under %s.', dataRoot);
end
trials = [trialCell{:}];
end

function meta = parseTrialName(pathStr)
[~, nameOnly, ~] = fileparts(pathStr);
nameL = lower(nameOnly);
if contains(nameL, 'parallel')
    topology = 'sealed';
else
    topology = 'coupled';
end

tok = regexp(nameL, '(axial|circular|triangular|triangle)_([123])-([0-9]+)_', 'tokens', 'once');
if isempty(tok)
    error('Could not parse trial metadata from filename: %s', pathStr);
end
waveform = tok{1};
if strcmp(waveform, 'triangle')
    waveform = 'triangular';
end
preinflationPsi = str2double(tok{2});
pmaxPsi = str2double(tok{3});
label = sprintf('%s_%d-%d_%s', waveform, preinflationPsi, pmaxPsi, topology);

meta = struct();
meta.path = pathStr;
meta.topology = topology;
meta.waveform = waveform;
meta.preinflation_psi = preinflationPsi;
meta.pmax_psi = pmaxPsi;
meta.label = label;
end

function trial = loadTrial(meta, opts)
try
    T = readtable(meta.path, 'VariableNamingRule', 'preserve');
catch
    T = readtable(meta.path, 'PreserveVariableNames', true);
end
A = tableColumnsToNumeric(T, opts.requiredCols);
A = A(all(isfinite(A), 2), :);
if isempty(A)
    error('No valid numeric rows remain after NaN removal: %s', meta.path);
end

A = sortrows(A, 1);
dtRaw = [1; diff(A(:, 1))];
A = A(dtRaw > 0, :);
if size(A, 1) <= opts.delay
    error('Too few rows after duplicate removal: %s', meta.path);
end

A(:, 1) = A(:, 1) - A(1, 1);
tMax = A(end, 1);
keep = (A(:, 1) >= opts.trimSec) & (A(:, 1) <= (tMax - opts.trimSec));
A = A(keep, :);
if size(A, 1) <= opts.delay
    error('Too few rows after trimming for %s.', meta.path);
end

time = A(:, 1);
inputs = A(:, 2:4);
sensors = A(:, 5:9);
q1 = A(:, 10:13);
q3 = A(:, 14:17);
thetaDeg = computeThetaDeg(q1, q3);
dt = median(diff(time));
sensorFeatures = buildDelayFeatureMatrix(sensors, opts.delay);

trial = struct();
trial.meta = meta;
trial.time = time;
trial.theta_deg = thetaDeg;
trial.sensors = sensors;
trial.inputs = inputs;
trial.dt = dt;
trial.sensor_features = sensorFeatures;
end

function A = tableColumnsToNumeric(T, requiredCols)
A = zeros(height(T), numel(requiredCols));
for j = 1:numel(requiredCols)
    name = requiredCols{j};
    if ~ismember(name, T.Properties.VariableNames)
        error('Required column not found: %s', name);
    end
    v = T.(name);
    if iscell(v)
        v = str2double(v);
    elseif isstring(v) || ischar(v)
        v = str2double(cellstr(v));
    else
        v = double(v);
    end
    A(:, j) = v;
end
end

function thetaDeg = computeThetaDeg(q1, q3)
q1 = normalizeQuaternions(q1);
q3 = normalizeQuaternions(q3);
qRel = quatMultiply(quatInverse(q1), q3);
qRelW = min(max(abs(qRel(:, 4)), 0.0), 1.0);
thetaDeg = rad2deg(2.0 .* acos(qRelW));
end

function qn = normalizeQuaternions(q)
n = sqrt(sum(q.^2, 2));
n(n <= 0) = 1;
qn = q ./ n;
end

function qi = quatInverse(q)
qi = q;
qi(:, 1:3) = -qi(:, 1:3);
end

function q = quatMultiply(q1, q2)
x1 = q1(:, 1); y1 = q1(:, 2); z1 = q1(:, 3); w1 = q1(:, 4);
x2 = q2(:, 1); y2 = q2(:, 2); z2 = q2(:, 3); w2 = q2(:, 4);
q = [ ...
    w1 .* x2 + x1 .* w2 + y1 .* z2 - z1 .* y2, ...
    w1 .* y2 - x1 .* z2 + y1 .* w2 + z1 .* x2, ...
    w1 .* z2 + x1 .* y2 - y1 .* x2 + z1 .* w2, ...
    w1 .* w2 - x1 .* x2 - y1 .* y2 - z1 .* z2];
end

function X = buildDelayFeatureMatrix(data, delay)
[n, d] = size(data);
if n < delay
    error('Need at least %d rows, got %d.', delay, n);
end
X = zeros(n - delay + 1, delay * d, 'double');
for lag = 1:delay
    cols = (lag - 1) * d + (1:d);
    X(:, cols) = data(delay - lag + 1 : n - lag + 1, :);
end
end

function [X, y, tFull] = alignSupervisedProblem(trial, subset, horizonSteps, opts)
if nargin < 4
    opts = defaultOptions();
end
if horizonSteps < 0
    error('HorizonSteps must be >= 0.');
end
cols = subsetFeatureColumns(subset, opts.delay, opts.totalPouches);
X = trial.sensor_features(:, cols);
y = trial.theta_deg(opts.delay + horizonSteps : end);
tFull = trial.time(opts.delay + horizonSteps : end);
if horizonSteps > 0
    X = X(1:end-horizonSteps, :);
end
if size(X, 1) ~= numel(y)
    error('Aligned feature/target lengths differ.');
end
y = y(:);
tFull = tFull(:);
end

function cols = subsetFeatureColumns(subset, delay, totalPouches)
subset = subset(:)';
cols = zeros(1, delay * numel(subset));
ptr = 0;
for lag = 0:(delay - 1)
    base = lag * totalPouches;
    idx = base + subset;
    cols(ptr + (1:numel(subset))) = idx;
    ptr = ptr + numel(subset);
end
end

function [yTest, yPred, idx] = fitRidgePredict(X, y, alpha, trainFraction)
if nargin < 3 || isempty(alpha)
    alpha = 0.01;
end
if nargin < 4 || isempty(trainFraction)
    trainFraction = 0.70;
end
n = size(X, 1);
split = floor(trainFraction * n);
if split <= 0 || split >= n
    error('Train/test split is empty.');
end

XTrain = double(X(1:split, :));
XTest = double(X(split+1:end, :));
yTrain = double(y(1:split, :));
yTest = double(y(split+1:end, :));

xMean = mean(XTrain, 1);
xStd = std(XTrain, 1, 1);
xStd(xStd < 1e-12) = 1.0;
XTrainZ = bsxfun(@rdivide, bsxfun(@minus, XTrain, xMean), xStd);
XTestZ = bsxfun(@rdivide, bsxfun(@minus, XTest, xMean), xStd);

yMean = mean(yTrain, 1);
YTrainC = bsxfun(@minus, yTrain, yMean);

A = XTrainZ' * XTrainZ;
A(1:size(A, 1)+1:end) = A(1:size(A, 1)+1:end) + alpha;
B = XTrainZ' * YTrainC;
coef = A \ B;
yPred = bsxfun(@plus, XTestZ * coef, yMean);
idx = (split+1:n)';
end

function pred = prcPrediction(trial, subset, opts, horizonSteps)
if nargin < 2 || isempty(subset)
    subset = 1:opts.totalPouches;
end
if nargin < 4
    horizonSteps = opts.horizonSteps;
end
[X, y, tFull] = alignSupervisedProblem(trial, subset, horizonSteps, opts);
[yTest, yPred, idx] = fitRidgePredict(X, y, opts.alpha, opts.trainFraction);
pred = struct();
pred.time = tFull(idx);
pred.y_true = yTest(:);
pred.y_pred = yPred(:);
pred.nmse = nmseMetric(pred.y_true, pred.y_pred);
pred.rmse_deg = sqrt(mean((pred.y_true - pred.y_pred).^2));
end

function val = nmseMetric(yTrue, yPred)
denom = sum((yTrue - mean(yTrue)).^2);
if denom <= 1e-12
    val = NaN;
else
    val = sum((yTrue - yPred).^2) / denom;
end
end

function sensorsOut = lowpassFilterColumns(sensors, dt, cutoffHz, order)
if nargin < 4 || isempty(order)
    order = 3;
end
fs = 1.0 / dt;
cutoff = min(cutoffHz, 0.45 * fs);
[b, a] = butter(order, cutoff / (0.5 * fs), 'low');
sensorsOut = zeros(size(sensors));
for k = 1:size(sensors, 2)
    sensorsOut(:, k) = filtfilt(b, a, sensors(:, k));
end
end

function [meanCorr, corrMat] = computePairwiseCorrStats(sensors)
corrMat = corrcoef(sensors);
mask = triu(true(size(corrMat)), 1);
vals = corrMat(mask);
meanCorr = mean(vals);
end

function val = meanPairwiseCCC(sensors)
vals = [];
for i = 1:size(sensors, 2)
    x = sensors(:, i);
    mux = mean(x);
    varx = var(x, 0, 1);
    for j = (i + 1):size(sensors, 2)
        y = sensors(:, j);
        muy = mean(y);
        vary = var(y, 0, 1);
        C = cov(x, y);
        covxy = C(1, 2);
        denom = varx + vary + (mux - muy)^2;
        if denom > 1e-12
            vals(end+1, 1) = (2.0 * covxy) / denom; %#ok<AGROW>
        end
    end
end
if isempty(vals)
    val = NaN;
else
    val = mean(vals);
end
end

function [eigVals, ratio] = pcaFromCovariance(sensors)
centered = bsxfun(@minus, sensors, mean(sensors, 1));
covMat = cov(centered);
eigVals = sort(real(eig(covMat)), 'descend');
eigVals(eigVals < 0) = 0;
if sum(eigVals) <= 0
    ratio = zeros(size(eigVals));
else
    ratio = eigVals / sum(eigVals);
end
end

function val = participationRatio(eigVals)
denom = sum(eigVals.^2);
if denom <= 1e-12
    val = NaN;
else
    val = (sum(eigVals)^2) / denom;
end
end

function val = meanPouchSigma(sensors)
val = mean(std(sensors, 0, 1));
end

function val = responseDiversity(sensors)
sigmas = std(sensors, 0, 1);
val = std(sigmas, 0, 2) / mean(sigmas);
end

function val = meanSensitivityPsiPerDeg(sensors, thetaDeg)
X = [thetaDeg(:), ones(numel(thetaDeg), 1)];
slopes = zeros(1, size(sensors, 2));
for k = 1:size(sensors, 2)
    coef = X \ sensors(:, k);
    slopes(k) = abs(coef(1));
end
val = mean(slopes);
end

function val = meanSNRLowpass(sensors, fsHz, cutoffHz)
if nargin < 3 || isempty(cutoffHz)
    cutoffHz = 5.0;
end
cutoff = min(cutoffHz, 0.45 * fsHz);
[b, a] = butter(4, cutoff / (0.5 * fsHz), 'low');
snrVals = NaN(1, size(sensors, 2));
for k = 1:size(sensors, 2)
    x = sensors(:, k);
    xFilt = filtfilt(b, a, x);
    resid = x - xFilt;
    varSignal = var(xFilt, 1, 1);
    varResid = var(resid, 1, 1);
    if varResid > 1e-12
        snrVals(k) = varSignal / varResid;
    end
end
val = mean(snrVals(~isnan(snrVals)));
end

function val = meanDynamicRange(sensors)
val = mean(max(sensors, [], 1) - min(sensors, [], 1));
end

function val = delayDecodingMemoryCapacity(trial, opts)
scores = zeros(opts.memoryK, 1);
for k = 1:opts.memoryK
    X = trial.sensors(1+k:end, :);
    Y = trial.inputs(1:end-k, :);
    [yTest, yPred] = fitRidgePredict(X, Y, opts.alpha, opts.trainFraction);
    perChannelR2 = [];
    for j = 1:size(Y, 2)
        yt = yTest(:, j);
        yp = yPred(:, j);
        ssTot = sum((yt - mean(yt)).^2);
        if ssTot <= 1e-12
            continue;
        end
        ssRes = sum((yt - yp).^2);
        perChannelR2(end+1, 1) = max(0.0, 1.0 - ssRes / ssTot); %#ok<AGROW>
    end
    if isempty(perChannelR2)
        scores(k) = 0.0;
    else
        scores(k) = mean(perChannelR2);
    end
end
val = sum(scores);
end

function [lagSec, curves] = crossCorrelationCurves(sensors, thetaDeg, dt, maxLagSec)
if nargin < 4 || isempty(maxLagSec)
    maxLagSec = 15.0;
end
maxLag = round(maxLagSec / dt);
n = size(sensors, 1);
curves = zeros(size(sensors, 2), 2 * maxLag + 1);
for i = 1:size(sensors, 2)
    x = sensors(:, i);
    y = thetaDeg(:);
    x = (x - mean(x)) / std(x, 1);
    y = (y - mean(y)) / std(y, 1);
    c = xcorr(x, y, maxLag, 'none') / n;
    curves(i, :) = c(:)';
end
lags = (-maxLag:maxLag)';
lagSec = lags * dt;
end

function cache = buildPredictionCache(trial, opts, horizonSteps)
if nargin < 3
    horizonSteps = opts.horizonSteps;
end
[XFull, yFull] = alignSupervisedProblem(trial, 1:opts.totalPouches, horizonSteps, opts);
n = size(XFull, 1);
split = floor(opts.trainFraction * n);
XTrain = double(XFull(1:split, :));
XTest = double(XFull(split+1:end, :));
yTrain = double(yFull(1:split, :));
yTest = double(yFull(split+1:end, :));

xMean = mean(XTrain, 1);
xStd = std(XTrain, 1, 1);
xStd(xStd < 1e-12) = 1.0;
XTrainZ = bsxfun(@rdivide, bsxfun(@minus, XTrain, xMean), xStd);
XTestZ = bsxfun(@rdivide, bsxfun(@minus, XTest, xMean), xStd);
yMean = mean(yTrain, 1);
yTrainC = bsxfun(@minus, yTrain, yMean);

cache = struct();
cache.gram = XTrainZ' * XTrainZ;
cache.rhs = XTrainZ' * yTrainC;
cache.x_test_z = XTestZ;
cache.y_test = yTest(:);
cache.y_mean = yMean;
end

function yPred = cachedSubsetPrediction(cache, subset, opts)
cols = subsetFeatureColumns(subset, opts.delay, opts.totalPouches);
A = cache.gram(cols, cols);
A(1:size(A, 1)+1:end) = A(1:size(A, 1)+1:end) + opts.alpha;
coef = A \ cache.rhs(cols, :);
yPred = cache.x_test_z(:, cols) * coef + cache.y_mean;
yPred = yPred(:);
end

function [med, elo, ehi] = quantileErrorbars(values)
values = values(:);
med = median(values);
q1 = linearQuantile(values, 0.25);
q3 = linearQuantile(values, 0.75);
elo = med - q1;
ehi = q3 - med;
end

function q = linearQuantile(x, p)
x = sort(x(:));
n = numel(x);
if n == 0
    q = NaN;
    return;
end
if p <= 0
    q = x(1);
    return;
end
if p >= 1
    q = x(end);
    return;
end
pos = 1 + (n - 1) * p;
lo = floor(pos);
hi = ceil(pos);
if lo == hi
    q = x(lo);
else
    w = pos - lo;
    q = (1 - w) * x(lo) + w * x(hi);
end
end

function metrics = buildTrialTable(trials, opts, horizonSteps)
nTrials = numel(trials);
trialCol = cell(nTrials, 1);
fileCol = cell(nTrials, 1);
topologyCol = cell(nTrials, 1);
waveformCol = cell(nTrials, 1);
preCol = zeros(nTrials, 1);
pmaxCol = zeros(nTrials, 1);
nmseCol = zeros(nTrials, 1);
rmseCol = zeros(nTrials, 1);
mdiCol = zeros(nTrials, 1);
meanCorrCol = zeros(nTrials, 1);
meanCccCol = zeros(nTrials, 1);
pc1VarCol = zeros(nTrials, 1);
prCol = zeros(nTrials, 1);
meanSigmaCol = zeros(nTrials, 1);
respDivCol = zeros(nTrials, 1);
meanSnrCol = zeros(nTrials, 1);
meanDynRangeCol = zeros(nTrials, 1);
meanSensCol = zeros(nTrials, 1);
thetaStdCol = zeros(nTrials, 1);

for i = 1:nTrials
    trial = trials(i);
    pred = prcPrediction(trial, 1:opts.totalPouches, opts, horizonSteps);
    [meanCorr, ~] = computePairwiseCorrStats(trial.sensors);
    [eigVals, ratio] = pcaFromCovariance(trial.sensors);

    trialCol{i} = trial.meta.label;
    [~, fileName, ext] = fileparts(trial.meta.path);
    fileCol{i} = [fileName, ext];
    topologyCol{i} = trial.meta.topology;
    waveformCol{i} = trial.meta.waveform;
    preCol(i) = trial.meta.preinflation_psi;
    pmaxCol(i) = trial.meta.pmax_psi;
    nmseCol(i) = pred.nmse;
    rmseCol(i) = pred.rmse_deg;
    mdiCol(i) = delayDecodingMemoryCapacity(trial, opts);
    meanCorrCol(i) = meanCorr;
    meanCccCol(i) = meanPairwiseCCC(trial.sensors);
    pc1VarCol(i) = 100.0 * ratio(1);
    prCol(i) = participationRatio(eigVals);
    meanSigmaCol(i) = meanPouchSigma(trial.sensors);
    respDivCol(i) = responseDiversity(trial.sensors);
    meanSnrCol(i) = meanSNRLowpass(trial.sensors, 1.0 / trial.dt, 5.0);
    meanDynRangeCol(i) = meanDynamicRange(trial.sensors);
    meanSensCol(i) = meanSensitivityPsiPerDeg(trial.sensors, trial.theta_deg);
    thetaStdCol(i) = std(trial.theta_deg, 0, 1);
end

metrics = table(trialCol, fileCol, topologyCol, waveformCol, preCol, pmaxCol, ...
    nmseCol, rmseCol, mdiCol, meanCorrCol, meanCccCol, pc1VarCol, prCol, ...
    meanSigmaCol, respDivCol, meanSnrCol, meanDynRangeCol, meanSensCol, thetaStdCol, ...
    'VariableNames', { ...
    'trial', 'file', 'topology', 'waveform', 'preinflation_psi', 'pmax_psi', ...
    'nmse', 'rmse_deg', 'delay_decoding_index', 'mean_inter_pouch_corr', ...
    'mean_ccc', 'pc1_variance_pct', 'participation_ratio', ...
    'mean_pouch_sigma_psi', 'response_diversity', 'mean_snr', ...
    'mean_dynamic_range_psi', 'mean_sensitivity_psi_per_deg', 'theta_std_deg'});

metrics = sortrows(metrics, {'topology', 'waveform', 'preinflation_psi', 'pmax_psi'});
end

function makeFig2(trials, metrics, outDir, opts)
exampleSealed = findSingleTrial(trials, 'sealed', 'triangular', 1, 5);
exampleCoupled = findSingleTrial(trials, 'coupled', 'triangular', 1, 5);

predSealed = prcPrediction(exampleSealed, 1:opts.totalPouches, opts, opts.horizonSteps);
predCoupled = prcPrediction(exampleCoupled, 1:opts.totalPouches, opts, opts.horizonSteps);

yPredSFilt = lowpassFilterColumns(predSealed.y_pred(:), exampleSealed.dt, 0.5, 3);
yPredCFilt = lowpassFilterColumns(predCoupled.y_pred(:), exampleCoupled.dt, 1.0, 3);

fig = figure('Color', 'w', 'Units', 'pixels', 'Position', [100, 40, 880, 1050]);
ax1 = subplot(3, 2, 1, 'Parent', fig);
ax2 = subplot(3, 2, 2, 'Parent', fig);
ax3 = subplot(3, 2, [3 4], 'Parent', fig);
ax4 = subplot(3, 2, [5 6], 'Parent', fig);
applyAxesStyle(ax1, opts);
applyAxesStyle(ax2, opts);
applyAxesStyle(ax3, opts);
applyAxesStyle(ax4, opts);

maskS = (predSealed.time - predSealed.time(1)) <= opts.fig2TraceSeconds;
tPlotS = predSealed.time(maskS) - predSealed.time(1);
plot(ax1, tPlotS, predSealed.y_true(maskS), 'LineWidth', opts.lineWidth, 'Color', opts.palette(1, :));
hold(ax1, 'on');
plot(ax1, tPlotS, yPredSFilt(maskS), 'LineWidth', opts.lineWidth, 'Color', [1, 0, 0]);
hold(ax1, 'off');
set(ax1, 'Box', 'on');
xlabel(ax1, 'Time (s)');
ylabel(ax1, '\theta (deg)');
title(ax1, '(a) Sealed');
legend(ax1, {'True', 'Estimate'}, 'Location', 'northeast');

maskC = (predCoupled.time - predCoupled.time(1)) <= opts.fig2TraceSeconds;
tPlotC = predCoupled.time(maskC) - predCoupled.time(1);
plot(ax2, tPlotC, predCoupled.y_true(maskC), 'LineWidth', opts.lineWidth, 'Color', opts.palette(1, :));
hold(ax2, 'on');
plot(ax2, tPlotC, yPredCFilt(maskC), 'LineWidth', opts.lineWidth, 'Color', [1, 0, 0]);
hold(ax2, 'off');
set(ax2, 'Box', 'on');
xlabel(ax2, 'Time (s)');
ylabel(ax2, '\theta (deg)');
title(ax2, '(b) Coupled');
legend(ax2, {'True', 'Estimate'}, 'Location', 'northeast');

boxOrder = { ...
    'axial', 'coupled'; ...
    'axial', 'sealed'; ...
    'circular', 'coupled'; ...
    'circular', 'sealed'; ...
    'triangular', 'coupled'; ...
    'triangular', 'sealed'};
boxPositions = [1, 2.5, 4.5, 6, 8, 9.5];
boxLabels = { ...
    sprintf('Axial\nCoupled'), ...
    sprintf('Axial\nSealed'), ...
    sprintf('Circular\nCoupled'), ...
    sprintf('Circular\nSealed'), ...
    sprintf('Triangular\nCoupled'), ...
    sprintf('Triangular\nSealed')};
boxData = cell(size(boxOrder, 1), 1);
for i = 1:size(boxOrder, 1)
    mask = strcmp(metrics.waveform, boxOrder{i, 1}) & strcmp(metrics.topology, boxOrder{i, 2});
    boxData{i} = metrics.nmse(mask);
end
plotCustomBoxGroups(ax3, boxData, boxPositions, boxLabels, opts.boxWidth, opts);
xlim(ax3, [0.2, 10.3]);
ylabel(ax3, 'NMSE');
title(ax3, '(c) Angle-estimation NMSE by waveform and topology');

mcData = {metrics.delay_decoding_index(strcmp(metrics.topology, 'coupled')), ...
          metrics.delay_decoding_index(strcmp(metrics.topology, 'sealed'))};
plotCustomBoxGroups(ax4, mcData, [1, 2], {'Coupled', 'Sealed'}, opts.boxWidth, opts);
xlabel(ax4, 'Topology');
ylabel(ax4, 'Memory Capacity (MC)');
title(ax4, '(d) Memory Capacity (K=40)');

saveFigure(fig, fullfile(outDir, 'fig2_performance'), opts.dpi);
close(fig);
end

function makeFig3(trials, outDir, opts)
coupled = findSingleTrial(trials, 'coupled', 'triangular', 1, 10);
sealed = findSingleTrial(trials, 'sealed', 'triangular', 1, 10);

[~, coupledCorr] = computePairwiseCorrStats(coupled.sensors);
[~, sealedCorr] = computePairwiseCorrStats(sealed.sensors);
[~, coupledRatio] = pcaFromCovariance(coupled.sensors);
[~, sealedRatio] = pcaFromCovariance(sealed.sensors);
[coupledLag, coupledXcorr] = crossCorrelationCurves(coupled.sensors, coupled.theta_deg, coupled.dt, opts.maxXcorrLagSec);
[sealedLag, sealedXcorr] = crossCorrelationCurves(sealed.sensors, sealed.theta_deg, sealed.dt, opts.maxXcorrLagSec);

fig = figure('Color', 'w', 'Units', 'pixels', 'Position', [120, 60, 1000, 900]);
ax1 = subplot(2, 2, 1, 'Parent', fig);
ax2 = subplot(2, 2, 2, 'Parent', fig);
ax3 = subplot(2, 2, 3, 'Parent', fig);
ax4 = subplot(2, 2, 4, 'Parent', fig);
applyAxesStyle(ax1, opts);
applyAxesStyle(ax2, opts);
applyAxesStyle(ax3, opts);
applyAxesStyle(ax4, opts);

imagesc(ax1, coupledCorr);
axis(ax1, 'image');
set(ax1, 'YDir', 'normal', 'XTick', 1:5, 'YTick', 1:5);
caxis(ax1, [0.5, 1.0]);
cb1 = colorbar(ax1);
set(cb1);
title(ax1, '(a) Coupled inter-pouch correlation');

imagesc(ax2, sealedCorr);
axis(ax2, 'image');
set(ax2, 'YDir', 'normal', 'XTick', 1:5, 'YTick', 1:5);
caxis(ax2, [0.5, 1.0]);
cb2 = colorbar(ax2);
set(cb2);
title(ax2, '(b) Sealed inter-pouch correlation');

pcs = 1:opts.totalPouches;
plot(ax3, pcs, cumsum(coupledRatio(:)), 'LineWidth', opts.lineWidth, 'Color', opts.topologyColors.coupled);
hold(ax3, 'on');
plot(ax3, pcs, cumsum(sealedRatio(:)), 'LineWidth', opts.lineWidth, 'Color', opts.topologyColors.sealed);
hold(ax3, 'off');
set(ax3, 'Box', 'on', 'YLim', [0.90, 1.0005]);
xlabel(ax3, 'Number of PCs');
ylabel(ax3, 'Cumulative variance');
title(ax3, '(c) PCA cumulative variance');
legend(ax3, {'Coupled', 'Sealed'}, 'Location', 'east');

pouchHandles = gobjects(5, 1);
for i = 1:opts.totalPouches
    pouchHandles(i) = plot(ax4, coupledLag, coupledXcorr(i, :), 'LineWidth', opts.lineWidth, 'Color', opts.palette(i, :));
    hold(ax4, 'on');
    plot(ax4, sealedLag, sealedXcorr(i, :), '--', 'LineWidth', opts.lineWidth, 'Color', opts.palette(i, :));
end
hold(ax4, 'off');
set(ax4, 'Box', 'on');
xlabel(ax4, 'Lag (s)    (positive: pouch leads)');
ylabel(ax4, 'Corr.');
title(ax4, '(d) Cross-correlation pouch vs \theta');
legPouch = legend(ax4, pouchHandles, {'P1', 'P2', 'P3', 'P4', 'P5'}, ...
    'Location', 'northwest');
set(legPouch, 'Box', 'on');
if isprop(legPouch, 'NumColumns')
    legPouch.NumColumns = 3;
end
styleSpecs(1) = struct('Color', opts.grayColor, 'LineStyle', '-', 'LineWidth', opts.lineWidth, ...
    'Marker', 'none', 'MarkerSize', opts.markerSize, 'MarkerFaceColor', 'none', 'MarkerEdgeColor', opts.grayColor); %#ok<AGROW>
styleSpecs(2) = struct('Color', opts.grayColor, 'LineStyle', '--', 'LineWidth', opts.lineWidth, ...
    'Marker', 'none', 'MarkerSize', opts.markerSize, 'MarkerFaceColor', 'none', 'MarkerEdgeColor', opts.grayColor); %#ok<AGROW>
createOverlayLegend(ax4, styleSpecs, {'Coupled', 'Sealed'}, 'southeast', '', opts);

saveFigure(fig, fullfile(outDir, 'fig3_diversity'), opts.dpi);
close(fig);
end

function makeFig4(metrics, outDir, opts)
fig = figure('Color', 'w', 'Units', 'pixels', 'Position', [120, 70, 860, 380]);

ax1 = subplot(1, 2, 1, 'Parent', fig);
ax2 = subplot(1, 2, 2, 'Parent', fig);
applyAxesStyle(ax1, opts);
applyAxesStyle(ax2, opts);

% --- Panel (a): Mean pouch sigma vs P0 ---
plotBaselineErrorbars(ax1, metrics, 'mean_pouch_sigma_psi', opts);
set(ax1, 'XTick', [1, 2, 3], 'XLim', [0.7, 3.3]);
xlabel(ax1, 'Baseline pressure P_0 (PSI)');
ylabel(ax1, 'Mean pouch \sigma (PSI)');
title(ax1, '(a)');
leg1 = legend(ax1, {'Coupled', 'Sealed'}, 'Location', 'northwest');
set(leg1, 'Box', 'on');

% --- Panel (b): NMSE vs P0 ---
plotBaselineErrorbars(ax2, metrics, 'nmse', opts);
set(ax2, 'XTick', [1, 2, 3], 'XLim', [0.7, 3.3]);
xlabel(ax2, 'Baseline pressure P_0 (PSI)');
ylabel(ax2, 'NMSE');
title(ax2, '(b)');
leg2 = legend(ax2, {'Coupled', 'Sealed'}, 'Location', 'northwest');
set(leg2, 'Box', 'on');

set(findall(fig, 'Type', 'axes'), 'Toolbar', []);
saveFigure(fig, fullfile(outDir, 'fig4_regime'), opts.dpi);
close(fig);
end

function plotBaselineErrorbars(ax, metrics, yCol, opts)
preVals = [1, 2, 3];
plotOneBaselineErrorbar(ax, metrics, 'coupled', yCol, preVals, opts);
hold(ax, 'on');
plotOneBaselineErrorbar(ax, metrics, 'sealed', yCol, preVals, opts);
hold(ax, 'off');
set(ax, 'Box', 'on');
end

function plotOneBaselineErrorbar(ax, metrics, topo, yCol, preVals, opts)
xs = zeros(1, numel(preVals));
ys = zeros(1, numel(preVals));
yLo = zeros(1, numel(preVals));
yHi = zeros(1, numel(preVals));
for i = 1:numel(preVals)
    mask = strcmp(metrics.topology, topo) & (metrics.preinflation_psi == preVals(i));
    vals = metrics{mask, yCol};
    [med, elo, ehi] = quantileErrorbars(vals);
    xs(i) = preVals(i);
    ys(i) = med;
    yLo(i) = elo;
    yHi(i) = ehi;
end
errorbar(ax, xs, ys, yLo, yHi, 'o-', 'Color', opts.topologyColors.(topo), 'LineWidth', opts.lineWidth, ...
    'MarkerSize', opts.markerSize, 'MarkerFaceColor', opts.topologyColors.(topo));
end

function plotMeanCccCurve(ax, metrics, topo, preVals, opts)
xs = zeros(1, numel(preVals));
ys = zeros(1, numel(preVals));
for i = 1:numel(preVals)
    mask = strcmp(metrics.topology, topo) & (metrics.preinflation_psi == preVals(i));
    vals = metrics.mean_ccc(mask);
    ys(i) = median(vals);
    xs(i) = preVals(i);
end
plot(ax, xs, ys, 'o-', 'Color', opts.topologyColors.(topo), 'LineWidth', opts.lineWidth, ...
    'MarkerSize', opts.markerSize, 'MarkerFaceColor', opts.topologyColors.(topo));
end

function fig5Summary = makeFig5(trials, outDir, opts)
sealedTrials = filterTrials(trials, 'sealed', '', [], []);
coupledTrials = filterTrials(trials, 'coupled', '', [], []);

[sBestFixed, sBestSubsets, sTrialNmseByM, subsetListByM] = evalTopology(sealedTrials, true, opts);
[cBestFixed, cBestSubsets] = evalTopology(coupledTrials, false, opts);

ms = 1:opts.totalPouches;
fullSubset = 1:opts.totalPouches;
fullIdx = findSubsetRow(subsetListByM{opts.totalPouches}, fullSubset);
fullNmse = sTrialNmseByM{opts.totalPouches}(:, fullIdx);
leaveOneOutDelta = zeros(1, opts.totalPouches);
for removed = 1:opts.totalPouches
    subset = setdiff(fullSubset, removed, 'stable');
    idx = findSubsetRow(subsetListByM{opts.totalPouches - 1}, subset);
    deltas = sTrialNmseByM{opts.totalPouches - 1}(:, idx) - fullNmse;
    leaveOneOutDelta(removed) = median(deltas);
end

marginalGain = sBestFixed(1:end-1) - sBestFixed(2:end);
totalGain = sBestFixed(1) - sBestFixed(end);
if totalGain > 1e-12
    recovered2 = marginalGain(1) / totalGain;
    recovered3 = sum(marginalGain(1:2)) / totalGain;
else
    recovered2 = 0.0;
    recovered3 = 0.0;
end

fig = figure('Color', 'w', 'Units', 'pixels', 'Position', [150, 120, 500, 450]);
ax = axes('Parent', fig);
applyAxesStyle(ax, opts);
plot(ax, ms, sBestFixed, 'o-', 'LineWidth', 1.6, 'MarkerSize', 3.5, 'Color', opts.topologyColors.sealed, ...
    'MarkerFaceColor', opts.topologyColors.sealed);
hold(ax, 'on');
plot(ax, ms, cBestFixed, 's-', 'LineWidth', 1.6, 'MarkerSize', 3.5, 'Color', opts.topologyColors.coupled, ...
    'MarkerFaceColor', opts.topologyColors.coupled);
hold(ax, 'off');
set(ax, 'Box', 'on', 'XTick', ms, 'YLim', [0.08, 1.02]);
grid(ax, 'on');
ax.GridAlpha = 0.25;
xlabel(ax, 'Number of instrumented sensors');
ylabel(ax, 'NMSE (median across trials)');
leg = legend(ax, {opts.sealedLabel, 'Coupled'}, 'Location', 'northeast');
set(leg, 'Box', 'on');

saveFigure(fig, fullfile(outDir, 'fig5_ablation'), opts.dpi);
close(fig);

fig5Summary = struct();
fig5Summary.best_fixed_subsets_1_indexed = struct( ...
    'count_1', sBestSubsets{1}, ...
    'count_2', sBestSubsets{2}, ...
    'count_3', sBestSubsets{3}, ...
    'count_4', sBestSubsets{4}, ...
    'count_5', sBestSubsets{5});
fig5Summary.best_fixed_median_nmse = struct( ...
    'count_1', sBestFixed(1), ...
    'count_2', sBestFixed(2), ...
    'count_3', sBestFixed(3), ...
    'count_4', sBestFixed(4), ...
    'count_5', sBestFixed(5));
fig5Summary.marginal_gain_median_nmse = struct( ...
    'step_1_to_2', marginalGain(1), ...
    'step_2_to_3', marginalGain(2), ...
    'step_3_to_4', marginalGain(3), ...
    'step_4_to_5', marginalGain(4));
fig5Summary.gain_fraction_recovered = struct('two_sensors', recovered2, 'three_sensors', recovered3);
fig5Summary.leave_one_out_delta_nmse_median = struct( ...
    'pouch_1', leaveOneOutDelta(1), ...
    'pouch_2', leaveOneOutDelta(2), ...
    'pouch_3', leaveOneOutDelta(3), ...
    'pouch_4', leaveOneOutDelta(4), ...
    'pouch_5', leaveOneOutDelta(5));
fig5Summary.sealed_best_curve = sBestFixed;
fig5Summary.coupled_best_curve = cBestFixed;
fig5Summary.coupled_best_subsets_1_indexed = struct( ...
    'count_1', cBestSubsets{1}, ...
    'count_2', cBestSubsets{2}, ...
    'count_3', cBestSubsets{3}, ...
    'count_4', cBestSubsets{4}, ...
    'count_5', cBestSubsets{5});
end

function [bestFixed, bestSubsets, trialNmseByM, subsetListByM] = evalTopology(topoTrials, isSealed, opts)
subsetListByM = cell(1, opts.totalPouches);
trialNmseByM = cell(1, opts.totalPouches);
subsetMedianByM = cell(1, opts.totalPouches);
cacheCell = cell(numel(topoTrials), 1);
for i = 1:numel(topoTrials)
    cacheCell{i} = buildPredictionCache(topoTrials(i), opts, opts.horizonSteps);
end
caches = [cacheCell{:}];

bestFixed = zeros(1, opts.totalPouches);
bestSubsets = cell(1, opts.totalPouches);
for m = 1:opts.totalPouches
    subsets = nchoosek(1:opts.totalPouches, m);
    nSub = size(subsets, 1);
    vals = zeros(numel(topoTrials), nSub);
    meds = zeros(nSub, 1);
    for s = 1:nSub
        subset = subsets(s, :);
        for i = 1:numel(topoTrials)
            yPred = cachedSubsetPrediction(caches(i), subset, opts);
            vals(i, s) = nmseMetric(caches(i).y_test, yPred);
        end
        meds(s) = median(vals(:, s));
    end
    bestIdx = find(meds == min(meds), 1, 'first');
    if isSealed && m == 1
        bestIdx = findSubsetRow(subsets, 5);
    end
    bestFixed(m) = meds(bestIdx);
    bestSubsets{m} = subsets(bestIdx, :);
    subsetListByM{m} = subsets;
    trialNmseByM{m} = vals;
    subsetMedianByM{m} = meds; %#ok<NASGU>
end
end

function idx = findSubsetRow(subsets, subset)
subset = subset(:)';
if size(subsets, 2) ~= numel(subset)
    error('Subset width mismatch.');
end
match = all(bsxfun(@eq, subsets, subset), 2);
idx = find(match, 1, 'first');
if isempty(idx)
    error('Requested subset was not found.');
end
end

function writeSummaryFiles(metrics, fig5Summary, outDir, opts)
writetable(metrics, fullfile(outDir, 'trial_metrics.csv'));

topoList = {'coupled', 'sealed'};
medianNmse = zeros(2, 1);
meanDDI = zeros(2, 1);
stdDDI = zeros(2, 1);
meanCorr = zeros(2, 1);
meanCcc = zeros(2, 1);
meanSens = zeros(2, 1);
for i = 1:2
    mask = strcmp(metrics.topology, topoList{i});
    medianNmse(i) = median(metrics.nmse(mask));
    meanDDI(i) = mean(metrics.delay_decoding_index(mask));
    stdDDI(i) = std(metrics.delay_decoding_index(mask), 0, 1);
    meanCorr(i) = mean(metrics.mean_inter_pouch_corr(mask));
    meanCcc(i) = mean(metrics.mean_ccc(mask));
    meanSens(i) = mean(metrics.mean_sensitivity_psi_per_deg(mask));
end

topologySummary = table(topoList(:), medianNmse, meanDDI, stdDDI, meanCorr, meanCcc, meanSens, ...
    'VariableNames', {'topology', 'median_nmse', 'mean_delay_decoding_index', ...
    'std_delay_decoding_index', 'mean_inter_pouch_corr', 'mean_ccc', 'mean_sensitivity_psi_per_deg'});
writetable(topologySummary, fullfile(outDir, 'topology_summary.csv'));

try
    jsonText = jsonencode(fig5Summary);
    fid = fopen(fullfile(outDir, 'fig5_subset_summary.json'), 'w');
    if fid >= 0
        fwrite(fid, jsonText, 'char');
        fclose(fid);
    end
catch
end

readmePath = fullfile(outDir, 'README_matlab_regen.txt');
fid = fopen(readmePath, 'w');
if fid >= 0
    fprintf(fid, 'Regenerated IFAC plots from MATLAB\n');
    fprintf(fid, 'Panel4bMode=%s\n', opts.panel4bMode);
    fprintf(fid, 'HorizonSteps=%d\n', opts.horizonSteps);
    fprintf(fid, 'DPI=%d\n', opts.dpi);
    fprintf(fid, 'Figure 5 uses the label %s for the sealed topology.\n', opts.sealedLabel);
    fclose(fid);
end
end

function trial = findSingleTrial(trials, topology, waveform, prePsi, pmaxPsi)
matches = filterTrials(trials, topology, waveform, prePsi, pmaxPsi);
if numel(matches) ~= 1
    error('Expected exactly one matching trial.');
end
trial = matches(1);
end

function out = filterTrials(trials, topology, waveform, prePsi, pmaxPsi)
keep = true(numel(trials), 1);
for i = 1:numel(trials)
    if ~isempty(topology)
        keep(i) = keep(i) && strcmp(trials(i).meta.topology, topology);
    end
    if ~isempty(waveform)
        keep(i) = keep(i) && strcmp(trials(i).meta.waveform, waveform);
    end
    if ~isempty(prePsi)
        keep(i) = keep(i) && (trials(i).meta.preinflation_psi == prePsi);
    end
    if ~isempty(pmaxPsi)
        keep(i) = keep(i) && (trials(i).meta.pmax_psi == pmaxPsi);
    end
end
out = trials(keep);
end

function plotCustomBoxGroups(ax, groups, positions, labels, width, opts)
cla(ax);
hold(ax, 'on');
for i = 1:numel(groups)
    drawCustomBox(ax, positions(i), groups{i}, width, opts);
end
hold(ax, 'off');
set(ax, 'Box', 'on', 'XTick', positions, 'XTickLabel', labels);
end

function drawCustomBox(ax, xPos, data, width, opts)
data = data(:);
data = data(~isnan(data));
if isempty(data)
    return;
end
stats = boxStatistics(data);
patch(ax, xPos + width * [-0.5, 0.5, 0.5, -0.5], [stats.q1, stats.q1, stats.q3, stats.q3], [1, 1, 1], ...
    'EdgeColor', 'k', 'LineWidth', 1.2);
line(ax, xPos + width * [-0.5, 0.5], [stats.median, stats.median], 'Color', opts.boxMedianColor, 'LineWidth', 1.4);
line(ax, [xPos, xPos], [stats.q3, stats.upperWhisker], 'Color', 'k', 'LineWidth', 1.1);
line(ax, [xPos, xPos], [stats.lowerWhisker, stats.q1], 'Color', 'k', 'LineWidth', 1.1);
line(ax, xPos + width * [-0.25, 0.25], [stats.upperWhisker, stats.upperWhisker], 'Color', 'k', 'LineWidth', 1.1);
line(ax, xPos + width * [-0.25, 0.25], [stats.lowerWhisker, stats.lowerWhisker], 'Color', 'k', 'LineWidth', 1.1);
if ~isempty(stats.outliers)
    plot(ax, xPos * ones(size(stats.outliers)), stats.outliers, 'ko', 'MarkerSize', 5, 'LineWidth', 1.0);
end
end

function stats = boxStatistics(x)
q1 = linearQuantile(x, 0.25);
med = linearQuantile(x, 0.50);
q3 = linearQuantile(x, 0.75);
iqrVal = q3 - q1;
lowFence = q1 - 1.5 * iqrVal;
highFence = q3 + 1.5 * iqrVal;
in = x >= lowFence & x <= highFence;
if any(in)
    lowerWhisker = min(x(in));
    upperWhisker = max(x(in));
else
    lowerWhisker = min(x);
    upperWhisker = max(x);
end
stats = struct('q1', q1, 'median', med, 'q3', q3, ...
    'lowerWhisker', lowerWhisker, 'upperWhisker', upperWhisker, 'outliers', x(~in));
end

function txt = subsetToString(subset)
subset = subset(:)';
if isempty(subset)
    txt = '{}';
    return;
end
parts = arrayfun(@(x) sprintf('%d', x), subset, 'UniformOutput', false);
txt = ['{', strjoin(parts, ','), '}'];
end

function saveFigure(fig, fileBase, dpi)
set(fig, 'PaperPositionMode', 'auto');
print(fig, [fileBase, '.pdf'], '-dpdf', '-painters');
print(fig, [fileBase, '.png'], '-dpng', sprintf('-r%d', dpi));
end

function applyAxesStyle(ax, ~)
set(ax, 'LineWidth', 1.0);
end

function createOverlayLegend(ax, specs, labels, location, titleText, opts)
fig = ancestor(ax, 'figure');
overlay = axes('Parent', fig, 'Position', get(ax, 'Position'), 'Color', 'none', ...
    'XColor', 'none', 'YColor', 'none', 'XTick', [], 'YTick', [], 'HitTest', 'off');
set(overlay, 'Visible', 'off');
hold(overlay, 'on');
handles = gobjects(numel(labels), 1);
for i = 1:numel(labels)
    s = specs(i);
    handles(i) = plot(overlay, NaN, NaN, 'Color', s.Color, 'LineStyle', s.LineStyle, ...
        'LineWidth', s.LineWidth, 'Marker', s.Marker, 'MarkerSize', s.MarkerSize, ...
        'MarkerFaceColor', s.MarkerFaceColor, 'MarkerEdgeColor', s.MarkerEdgeColor);
end
hold(overlay, 'off');
leg = legend(overlay, handles, labels, 'Location', location);
set(leg, 'Box', 'on');
if ~isempty(titleText)
    setLegendTitleCompat(leg, titleText, opts);
end
end

function setLegendTitleCompat(leg, titleText, ~)
try
    title(leg, titleText);
catch
    try
        t = get(leg, 'Title');
        set(t, 'String', titleText);
    catch
    end
end
end

function rgb = hex2rgb(hex)
hex = char(hex);
if hex(1) == '#'
    hex = hex(2:end);
end
if numel(hex) ~= 6
    error('Expected a 6-character hex color.');
end
rgb = [hex2dec(hex(1:2)), hex2dec(hex(3:4)), hex2dec(hex(5:6))] / 255.0;
end