function status = Save_Reachset_Bulk(R_total, varargin)
    % SAVE_REACHSET_BULK Summary of this function goes here
    %   Detailed explanation goes here

    %% Parse Input arguments
    default_savejson = false;
    default_outpath = './Saved_Primitives/';
    default_suffix = '';
    validation_outpath = @(x) isa(x, 'char');
    validation_savejson = @(x) isa(x, 'logical');
    validation_suffix = @(x) isa(x, 'char');
    parser = inputParser;

    addRequired(parser, 'R_total');
    addParameter(parser, 'fname', 'Primitives');
    addParameter(parser, 'savejson', default_savejson, validation_savejson);
    addParameter(parser, 'outpath', default_outpath, validation_outpath);
    addParameter(parser, 'suffix', default_suffix, validation_suffix);

    parse(parser, R_total, varargin{:});
    R_total = parser.Results.R_total;
    savejson = parser.Results.savejson;
    outpath = parser.Results.outpath;
    suffix = parser.Results.suffix;
    fname = parser.Results.fname;

    %% Generate the Outfilename
    % Outfilespec = 'jerk_%4.2f_Iv_step_%4.4f_a0_%4.2f_v0_%4.2f';
    % Outfilename=sprintf(Outfilespec, NameParams(1), NameParams(2), NameParams(3), NameParams(4));
    Outfilename = fname;
    Outfilename = strcat(Outfilename, suffix);

    %% Check if outpath exists and if not create it
    if ~exist(outpath, 'dir')
        mkdir(outpath);
    end
    matpath = strcat(outpath, 'mat/');
    if ~exist(matpath, 'dir')
        mkdir(matpath);
    end
    %% Save the file as a .mat
    save(strcat(matpath, strcat(Outfilename, '.mat')), "R_total");

    %% Save as json if wanted
    if savejson
        if ~exist(strcat(outpath, 'JSON'), 'dir')
            mkdir(strcat(outpath, 'JSON'));
        end
        JSONPATH = strcat(outpath, 'JSON/');
        fid = fopen(strcat(JSONPATH, strcat(Outfilename, '.json')), 'w+');
        fprintf(fid, '%s', jsonencode(R_total));
        fclose(fid);
    end

end
