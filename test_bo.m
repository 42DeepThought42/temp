% test_bo.m - smoke + correctness test for the bayes_optimize package.
% Run from the bayes/ directory:  octave test_bo.m
addpath(fileparts(mfilename('fullpath')));

disp('=== Test 1: 2-D Rosenbrock, minimize, EI ===');
bounds = [0 0; 2 2];
opt = struct('maxIter',15,'initialPoints',8,'seed',42,'verbose',true);
rosen = @(v) 100*(v(2)-v(1)^2)^2 + (1-v(1))^2;
[f, x, info] = bayes_optimize(rosen, 2, bounds, opt);
fprintf('best f = %.6e at x = [%.4f %.4f]\n', f, x(1), x(2));
fprintf('n evals = %d, bestIter = %d\n', info.nEvals, info.bestIter);
assert(f < 5, 'Rosenbrock should reach < 5');

disp('');
disp('=== Test 2: 2-D sphere, minimize, EI ===');
sphere = @(v) sum(v.^2);
[f, x, ~] = bayes_optimize(sphere, 2, [-5 -5; 5 5], ...
                            struct('maxIter',12,'initialPoints',8,'seed',1));
fprintf('best f = %.6e at x = [%.4f %.4f]\n', f, x(1), x(2));
assert(f < 1, 'Sphere should reach < 1');

disp('');
disp('=== Test 3: 3-D maximize, UCB ===');
neg_sphere = @(v) -sum(v.^2);
[f, x, ~] = bayes_optimize(neg_sphere, 3, [-3 -3 -3; 3 3 3], ...
                            struct('mode','maximize','maxIter',12,'initialPoints',9,'seed',7,'acquisition','UCB'));
fprintf('best f = %.6e at x = [%.4f %.4f %.4f]\n', f, x(1), x(2), x(3));
assert(f > -1, 'Maximize -sphere should reach > -1');

disp('');
disp('=== Test 4: name/value options form ===');
f1d = @(v) (v(1)-0.3)^2;
[fb, xb, ~] = bayes_optimize(f1d, 1, [0; 1], ...
                            'maxIter', 8, 'initialPoints', 6, 'seed', 3, ...
                            'acquisition', 'EI', 'verbose', false);
fprintf('best f = %.6e at x = %.4f (min near 0.3)\n', fb, xb(1));

disp('');
disp('=== Test 5: stateful objective {fn, x0} ===');
[fb, xb, ~] = bayes_optimize({@stateful_obj, 0.25}, 1, [0; 1], ...
                            'maxIter', 8, 'initialPoints', 6, 'seed', 5, 'verbose', false);
fprintf('best f = %.6e at x = %.4f (min near 0.25)\n', fb, xb(1));
assert(fb < 0.05, 'Stateful should get close to 0.25');

disp('');
disp('=== Test 6: fixed points ===');
fp = [0.1 0.9];
fv = [(0.1-0.3)^2; (0.9-0.3)^2];
f1d = @(v) (v(1)-0.3)^2;
[fb, xb, ~] = bayes_optimize(f1d, 1, [0; 1], ...
                            'maxIter', 8, 'initialPoints', 4, 'seed', 9, ...
                            'fixedPoints', fp, 'fixedValues', fv, 'verbose', false);
fprintf('best f = %.6e at x = %.4f\n', fb, xb(1));

disp('');
disp('=== Test 7: RBF kernel + LCB ===');
sphere = @(v) sum(v.^2);
[fb, xb, ~] = bayes_optimize(sphere, 2, [-2 -2; 2 2], ...
                            'maxIter', 8, 'initialPoints', 6, 'seed', 2, ...
                            'acquisition', 'LCB', 'gp', struct('kernel','rbf'), ...
                            'verbose', false);
fprintf('RBF+LCB: best f = %.6e\n', fb);

disp('');
disp('=== Test 8: entropy acquisition ===');
sphere = @(v) sum(v.^2);
[fb, xb, ~] = bayes_optimize(sphere, 2, [-2 -2; 2 2], ...
                            'maxIter', 8, 'initialPoints', 6, 'seed', 2, ...
                            'acquisition', 'entropy', 'verbose', false);
fprintf('entropy: best f = %.6e\n', fb);

disp('');
disp('=== Test 9: 5-D Ackley, known global minimum f(0)=0 on [-5,5]^5 ===');
% Uses the reference SFU Ackley (ackley_example.m): a smooth, BOUNDED function
% f = -20*exp(-0.2*sqrt(sum(x^2)/5)) - exp(sum(cos(2*pi*x))/5) + 20 + exp(1)
% with global minimum f(0,...,0) = 0.  It is highly multimodal in 5-D, so BO
% converges toward the minimum and typically settles in a local basin; we assert
% it makes strong progress (well below the ~6-13 range of the basin floor).
% (Early stopping is left disabled: default stopTolerance=0.)
ack = @(x) ackley_example(x);   % a=20, b=0.2, c=2*pi (defaults)
[f, x, info] = bayes_optimize(ack, 5, [-5*ones(1,5); 5*ones(1,5)], ...
                            struct('maxIter', 60, 'initialPoints', 25, ...
                                   'seed', 11, 'acquisition', 'EI', ...
                                   'nStarts', 150, 'verbose', true));
fprintf('best f = %.6e at x = [%s]\n', f, ...
        strjoin(arrayfun(@(t) sprintf('%.4f', t), x, 'UniformOutput', false), ' '));
fprintf('n evals = %d (initial 25 + up to 60 iterations)\n', info.nEvals);
assert(f < 2.0, '5-D Ackley should converge toward the known minimum f=0 (got f=%.4e)', f);

disp('');
disp('ALL TESTS PASSED');
