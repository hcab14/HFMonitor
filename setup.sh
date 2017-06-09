
P=`pwd`
export PYTHONPATH=$P/python:$PYTHONPATH

[ -d $P/software/install/lib ] && export LD_LIBRARY_PATH=$P/software/install/lib:$P/software/install/lib64/:$LD_LIBRARY_PATH
[ -d $P/bin ] && export PATH=$P/bin:$PATH
