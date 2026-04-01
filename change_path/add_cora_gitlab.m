function add_cora_gitlab()
    add_to_path_with_subfolders("/home/sebastian/Documents/Tools/Cora-Gitlab");
    add_cora_old_dependencies();
    rmpath(genpath("/home/sebastian/Documents/Tools/tbxmanager/toolboxes/mpt"));
    rmpath(genpath("/home/sebastian/Documents/Tools/tbxmanager/toolboxes/mptdoc"));
end
