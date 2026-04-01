function options = create_cora_options_1()
    % options.taylorTerms = 20;
    % options.zonotopeOrder = 100;
    % options.alg = 'lin-adaptive';
    % options.tensorOrder = 3;
    % options.errorOrder = 25;
    % options.intermediateOrder = 20;
    % options.timeStep = 0.01;

    options.taylorTerms = 20;
    options.zonotopeOrder = 50; % 100, 20
    options.alg = 'lin';
    options.tensorOrder = 2; % 2
    options.tensorOrderOutput = 2;
    options.errorOrder = 25;
    options.intermediateOrder = 20;
    options.timeStep = 0.01;
    %% options.reductionTechnique = 'pca'; %% FIXME test once again

    % options.algInner = 'minkdiff';
    %% Hint: setting the timeStepInner to 0.001 could maybe prevent
    %% empty inner reachsets (according to Max), but it does currently not
    %% work as systems with inputs are not really supported yet (only autonomous systems)
    %% - some weird errors occur
    % options.timeStepInner = 0.01; %'end';
    % options.orderInner = 50;
    % options.inpChanges = 10;

    % FIXME
    % max_error = zeros(30, 1);
    % max_error(:, 1) = inf;
    % max_error(2, 1) = 0.01; % y-error
    % options.maxError = max_error;
    % These settings performs much worse than the setting above:
    % options.alg = 'lin-adaptive';
    % tested these options; both completely useless
    % options.timeStep = 0.0001;
    %% FIXME
    % options.lagrangeRem.simplify = 'simplify';
end
