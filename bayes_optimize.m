function [fOpt, xOpt, info] = bayes_optimize(varargin)
%BAYES_OPTIMIZE  General-purpose Bayesian optimisation (base MATLAB only).
%
%   [fOpt, xOpt, info] = bayes_optIMIZE(fun, nvars, bounds, options)
%
%   Minimises (or maximises) an arbitrary, possibly non-smooth / black-box
%   objective using a Gaussian-process surrogate and an acquisition function.
%   No toolboxes are required.
%
%   INPUTS
%   ------
%   fun      Objective.  One of:
%                (a) function handle   f = fun(x)            (vectorised or not)
%                (b) {fn, x0}          f = fn(x, x0)         (stateful objective)
%   nvars    Number of scalar decision variables (1 x 1).
%   bounds   2-by-nvars matrix [lower; upper].
%   options  (optional) Either a STRUCTURE of options, or name/value pairs.
%            Any missing option is set to its default. See BOGETOPTIONS.
%
%   OUTPUTS
%   -------
%   fOpt     Best objective value found (on the ORIGINAL, unnormalised scale).
%   xOpt     Best decision vector (on the ORIGINAL scale), row vector 1-by-nvars.
%   info     Structure with fields:
%                .X       n-by-nvars   evaluated points (original scale)
%                .F       n-by-1       objective values
%                .history n-by-1       best value so far after each iteration
%                .bestIter index of the best evaluation
%                .nEvals  number of objective evaluations
%
%   EXAMPLE
%   ------
%   % 2-D problem, minimise Rosenbrock on a box
%   bounds = [0 0; 2 2];
%   [f, x] = bayes_optimize(@(v) rosenbrock(v), 2, bounds);
%
%   % with options as a struct
%   opt = struct('mode','minimize','maxIter',60,'initialPoints',12,'seed',1);
%   [f, x] = bayes_optimize(@(v) myLoss(v, myState), 5, bounds, opt);

    % ---- Parse positional args ---------------------------------------------
    if nargin < 3
        error('bayes_optimize:usage', ...
            'Usage: bayes_optimize(fun, nvars, bounds, options)');
    end
    fun    = varargin{1};
    nvars  = varargin{2};
    bounds = varargin{3};

    % ---- Parse options (struct or name/value) ------------------------------
    [nvars, bounds] = validateProblemInputs(nvars, bounds);
    if nargin >= 4
        if isstruct(varargin{4})
            opt = boGetOptions(varargin{4});
        else
            opt = boGetOptions(varargin(4:end));
        end
    else
        opt = boGetOptions();
    end

    % ---- Solve in the unit hyper-cube ----------------------------------------
    [fOpt01, xOpt01, info01] = boCoreLoop(fun, nvars, bounds, opt);

    % ---- Map back to the original variable space ----------------------------
    xOpt = boDenormalise(xOpt01, bounds);
    fOpt = info01.F(info01.bestIter);
    info = struct('X', boDenormalise(info01.X, bounds), ...
                  'F', info01.F, ...
                  'history', info01.history, ...
                  'bestIter', info01.bestIter, ...
                  'nEvals', info01.nEvals);
end

%% --------------------------------------------------------------------------
function [nvars, bounds] = validateProblemInputs(nvars, bounds)
    if ~isscalar(nvars) || nvars < 1 || nvars ~= floor(nvars)
        error('bayes_optimize:badInput', 'nvars must be a positive integer.');
    end
    nvars = round(nvars);
    if ~ismatrix(bounds) || any(size(bounds) ~= [2 nvars])
        error('bayes_optimize:badInput', ...
            'bounds must be a 2-by-nvars matrix [lower; upper].');
    end
    bounds = reshape(bounds, 2, nvars);
    if ~all(isfinite(bounds(:)))
        error('bayes_optimize:badInput', 'bounds must contain only finite values.');
    end
    if any(bounds(2,:) <= bounds(1,:))
        error('bayes_optimize:badInput', 'bounds(2,:) must be strictly > bounds(1,:).');
    end
end