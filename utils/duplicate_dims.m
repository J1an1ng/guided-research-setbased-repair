function res = duplicate_dims(cora_set, dims)
    % FIXME: we can actually replace duplicate_dims with project

    % We could also use project and then cartProd. However, for
    % conZonotopes, this would duplicate the constraints.
    % --> no, we cannot use cartProd, as this would combine each
    % state with each other state, which is not what we want.
    if isa(cora_set, 'zonotope') || isa(cora_set, 'conZonotope')
        c = center(cora_set);
        gens = generators(cora_set);
        new_c = [c; c(dims)];
        new_gens = [gens; gens(dims, :)];
        if isa(cora_set, 'zonotope')
            res = zonotope(new_c, new_gens);
        elseif isa(cora_set, 'conZonotope')
            res = conZonotope(new_c, new_gens, cora_set.A, cora_set.b);
        else
            error('Unexpected error.');
        end
    else
        % fprintf('duplication with cart prod\n');
        % res = cartProd(cora_set, project(cora_set, dims));
        error("Unsupported class for cora_set: %s.", class(cora_set));
    end
end
