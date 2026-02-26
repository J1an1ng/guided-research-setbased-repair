function add_cora_old_dependencies()
    %% We assume that tbxmanager was used to install all required packages
    %% apart from yalmip
    tbxmanager restorepath;
    add_to_path_with_subfolders("/home/sebastian/Documents/Tools/YALMIP-R20230609");
    add_to_path_with_subfolders("/home/sebastian/Documents/Tools/mosek");
end
