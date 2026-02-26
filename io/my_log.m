function my_log(fid, debug, s, varargin)
    if fid ~= -1
        fprintf(fid, s, varargin{:});
    end
    if debug
        fprintf(s, varargin{:});
    end
end
