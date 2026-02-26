function deactivate_cora_warning()
    %% Cora raised a deprecation warning when accessing zonotope.Z.
    %% However, this property is sometimes accessed automatically,
    %% e.g. during saving or loading.
    %% Add the warning-identifier below to CORA at the appropriate position
    %% in the code to make the suppressing work.
    warning('off', 'CORA:zonotopeZDeprecated');
    warning('off', 'CORA:zonotopeHalfspaceDeprecated');
    warning('off', 'CORA:redundantOptions');
end
