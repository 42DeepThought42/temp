function opt = boGetOptions(extra)
%BOGETOPTIONS  Build a validated options structure for bayes_optimize.
%
%   opt = boGetOPTIONS(extra)
%
%   Returns a fully-populated, validated options struct.  `extra` is the
%   trailing argument to bayes_optimize: it may be a STRUCTURE of options
%   (preferred) or one or more name/value pairs.  Missing fields default.
%   If called with no argument, defaults are returned.
%
%   Option fields and defaults
%   --------------------------
%   mode             'minimize' | 'maximize'      direction of optimisation
%   maxIter          30        number of Bayesian-optimisation iterations
%   initialPoints    10        size of the initial space-filling design
%   nStarts          50        multistart restarts for acquisition maximisation
%   acquisition      'EI'      'EI','PI','UCB','LCB','entropy'
%   kappa            2.57      exploration term (UCB/LCB)
%   xi               0.01     improvement term (EI/PI)
%   normalizeY       true      normalise objective for the GP (improves scaling)
%   initialMethod    'lhs'     'lhs' | 'random'
%   seed             0         random seed (0 = do not reset)
%   fixedPoints      []        2-by-k matrix of points already known to be
%                              good; used as extra training data (original scale)
%   fixedValues      []        k-by-1 objective values matching fixedPoints
%   verbose          true      print an iteration table
%   stopTolerance    0         early stop if best improves by < tol for
%                              'plateauLength' iterations.  A relative
%                              fraction (0 < tol < 1).  0 disables early
%                              stopping entirely.
%   plateauLength    10        patience in iterations for early stopping
%   gp               struct()  GP hyper-parameter overrides/limits, fields:
%                              lengthscales, lengthscaleBounds, signalVar,
%                              noiseVar, noiseVarBounds, maxIter (L-BFGS), tol

    % Defaults (order defines the "known fields" list used below).
    d = struct( ...
        'mode',            'minimize', ...
        'maxIter',         30, ...
        'initialPoints',   10, ...
        'nStarts',         50, ...
        'acquisition',     'EI', ...
        'kappa',           2.57, ...
        'xi',              0.01, ...
        'normalizeY',      true, ...
        'initialMethod',   'lhs', ...
        'seed',            0, ...
        'fixedPoints',     [], ...
        'fixedValues',     [], ...
        'verbose',         true, ...
        'stopTolerance',   0, ...
        'plateauLength',   10, ...
        'gp',              struct('kernel','matern32', 'lengthscales',[], ...
                                  'lengthscaleBounds',[], 'signalVar',[], ...
                                  'noiseVar',[], 'noiseVarBounds',[], ...
                                  'maxIter',[], 'tol',[]));

    % --- Convert name/value pairs into a struct, or accept a struct ----------
    if nargin >= 1 && ~isempty(extra)
        if isstruct(extra)
            s = extra;
        else
            % Name/value pairs: extra is a cell array {name, value, name, value, ...}
            if ~iscell(extra)
                extra = {extra};
            end
            if mod(numel(extra), 2) ~= 0
                error('boGetOptions:badInput', ...
                    'name/value options must be given in pairs.');
            end
            s = struct();
            for k = 1:2:numel(extra)
                if ~ischar(extra{k})
                    error('boGetOptions:badInput', ...
                        'option names must be character vectors.');
                end
                s.(extra{k}) = extra{k+1};
            end
        end
    else
        s = struct();
    end

    % --- Merge user fields onto defaults --------------------------------------
    fn = fieldnames(s);
    for i = 1:numel(fn)
        f = fn{i};
        if isfield(d, f)
            d.(f) = s.(f);
        else
            warning('boGetOptions:unknownField', ...
                ['Ignoring unknown option "%%s"; known fields: %s.'], f, ...
                strjoin(fieldnames(d), ', '));
        end
    end
    opt = d;

    % --- Validation -----------------------------------------------------------
    opt.mode = lower(char(opt.mode));
    if ~ismember(opt.mode, {'minimize','maximize'})
        error('boGetOptions:badMode','mode must be "minimize" or "maximize".');
    end

    opt.acquisition = upper(char(opt.acquisition));
    if ~ismember(opt.acquisition, {'EI','PI','UCB','LCB','ENTROPY'})
        error('boGetOptions:badAcq', ...
            'acquisition must be one of EI, PI, UCB, LCB, entropy.');
    end

    opt.initialMethod = lower(char(opt.initialMethod));
    if ~ismember(opt.initialMethod, {'lhs','random'})
        error('boGetOptions:badInit','initialMethod must be "lhs" or "random".');
    end

    boPosInt('maxIter',       opt.maxIter);
    boPosInt('initialPoints', opt.initialPoints);
    boPosInt('nStarts',       opt.nStarts);
    boPosInt('plateauLength', opt.plateauLength);

    opt.kappa         = boScalar('kappa', opt.kappa);
    opt.xi            = boScalar('xi',    opt.xi);
    opt.stopTolerance = boScalar('stopTolerance', opt.stopTolerance);
    if opt.stopTolerance < 0
        error('boGetOptions:badValue', 'stopTolerance must be >= 0.');
    end
    opt.seed          = boScalar('seed',  opt.seed);
    opt.normalizeY    = logical(opt.normalizeY);
    opt.verbose       = logical(opt.verbose);

    % GP sub-options (accept missing/empty = "use default").
    gp = opt.gp;
    if ~isfield(gp,'kernel') || isempty(gp.kernel), gp.kernel = 'matern32'; end
    if ~isfield(gp,'lengthscales'),       gp.lengthscales = []; end
    if ~isfield(gp,'lengthscaleBounds'),  gp.lengthscaleBounds = []; end
    if ~isfield(gp,'signalVar'),          gp.signalVar = []; end
    if ~isfield(gp,'noiseVar'),           gp.noiseVar = []; end
    if ~isfield(gp,'noiseVarBounds'),     gp.noiseVarBounds = []; end
    if ~isfield(gp,'maxIter'),            gp.maxIter = []; end
    if ~isfield(gp,'tol'),                gp.tol = []; end
    opt.gp = gp;

    % fixedPoints / fixedValues consistency
    if ~isempty(opt.fixedPoints)
        opt.fixedPoints = opt.fixedPoints(:);
        opt.fixedValues = opt.fixedValues(:);
        if isempty(opt.fixedValues)
            error('boGetOptions:badFixed','fixedValues must match fixedPoints.');
        end
        if numel(opt.fixedPoints) ~= numel(opt.fixedValues) ...
           || mod(numel(opt.fixedPoints),2) ~= 0
            error('boGetOptions:badFixed', ...
                'fixedPoints must be 2-by-k and fixedValues 1-by-k.');
        end
    end
end

%% --------------------------------------------------------------------------
function v = boPosInt(name, v)
    if ~isscalar(v) || v < 1 || v ~= floor(v)
        error('boGetOptions:badValue', ['%s must be a positive integer.'], name);
    end
    v = round(v);
end
function v = boScalar(name, v)
    if ~isscalar(v) || ~isfinite(v)
        error('boGetOptions:badValue', ['%s must be a finite scalar.'], name);
    end
end