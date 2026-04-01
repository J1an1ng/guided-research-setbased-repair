function boolout = Check_BoundaryCond(ReachSet, KSBounds, varargin) % mh:ignore_style
    % CHECK_BOUNDARYCOND Check whether the reachable set is contained within
    % the state boundaries
    %   Input:  reachSet    ReachSet;
    %           Boundaries  KSBounds
    %           varargin    array of Dimensions in which to bound the states
    %

    %% Default return: True (Boundaries were not overstepped)
    boolout = 1;

    parser = inputParser;

    addRequired(parser, 'ReachSet', @(x) isequal(class(x), 'reachSet'));
    addRequired(parser, 'KSBounds', @(x) isequal(class(x), 'KSBounds'));
    addOptional(parser, 'CheckDimensions', [KSTWithTrailerState.V, ...
                                            KSTWithTrailerState.A, KSTWithTrailerState.DELTA]);

    parse(parser, ReachSet, KSBounds, varargin{:});

    ReachSet = parser.Results.ReachSet;
    KSBounds = parser.Results.KSBounds;
    CheckDimensions = sort(parser.Results.CheckDimensions);

    %% Create Bounding Zonotope
    lb = [];
    ub = [];
    if any(CheckDimensions == 3) % steering angle constraint
        lb(end + 1) = (KSBounds.d_bounds(1));
        ub(end + 1) = (KSBounds.d_bounds(2));
    end
    if any(CheckDimensions == 4) % velocity constraint
        lb(end + 1) = (KSBounds.v_bounds(1));
        ub(end + 1) = (KSBounds.v_bounds(2));
    end
    if any(CheckDimensions == 7) % acceleration constraint
        lb(end + 1) = (KSBounds.a_bounds(1));
        ub(end + 1) = (KSBounds.a_bounds(2));
    end
    lb = lb';
    ub = ub';

    BoundingZonotope = interval(lb, ub);

    %% Step through all zonotopes in reachSet and check
    for ii = 1:length(ReachSet.timeInterval.set)
        Z_curr = project(ReachSet.timeInterval.set{ii}, CheckDimensions);
        if ~contains(BoundingZonotope, Z_curr)
            boolout = 0;
            break
        end
    end

end
