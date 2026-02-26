function options = create_cora_options_poly_1()
    options.taylorTerms = 10;
    options.zonotopeOrder = 20; % 50;
    options.alg = 'lin';
    options.tensorOrder = 3;
    options.tensorOrderOutput = 2;
    options.errorOrder = 25;
    options.intermediateOrder = 20;
    options.timeStep = 0.01;
    %% options.reductionTechnique = 'pca'; %% FIXME test once again
end
