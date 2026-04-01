function add_latest_cora()
    add_to_path_with_subfolders("/home/sebastian/Documents/Tools/Cora-Github/CORA");
    add_cora_old_dependencies();
    rmpath(genpath("/home/sebastian/Documents/Tools/tbxmanager/toolboxes/mpt"));
    rmpath(genpath("/home/sebastian/Documents/Tools/tbxmanager/toolboxes/mptdoc"));
end
