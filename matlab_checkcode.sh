#!/bin/bash

# Check if MATLAB is available
if ! command -v matlab &> /dev/null; then
    echo "ℹ️  MATLAB is not available. Skipping MATLAB code checks."
    echo "   To enable MATLAB code checks, ensure MATLAB is installed and accessible in your PATH."
    echo "   You can add MATLAB to your PATH by adding the MATLAB bin directory to your PATH variable."
    echo "   Example: export PATH=/path/to/matlab/bin:\$PATH"
    exit 0
fi

# FIXME this is currently necessary for me somehow, as matlab is not known otherwise
export PATH=~/MATLAB2023b/bin:$PATH

printf -v FILES "'%s'," "${@}"
FILES="${FILES%,}"

RES_FILE=pre_commit_res.txt
LOG_FILE=pre_commit_log.txt

MATLABCMD1="res = checkcode(${FILES}); if ~iscell(res); res = {res}; end; fid = fopen('${RES_FILE}','wt');fprintf(fid, int2str(sum(cellfun(@(x) size(x, 1), res))));fclose(fid); exit"
MATLABCMD2="diary('${LOG_FILE}'); checkcode(${FILES}); diary off; exit"

_=$(matlab -nodisplay -r "$MATLABCMD1")

if [ -f "$RES_FILE" ];
then
    res=$(<$RES_FILE)
else
    res='0'
fi

if [ "$res" -eq '0' ]
    then
        rm $RES_FILE
        exit 0
fi
_=$(matlab -nodisplay -r "$MATLABCMD2")
cat $LOG_FILE
rm $RES_FILE
rm $LOG_FILE
exit 1
