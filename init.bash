echo "adicionado $PWD/lib no LD_LIBRARY_PATH"
export LD_LIBRARY_PATH=$PWD/lib/:$LD_LIBRARY_PATH

echo "adicionado $PWD/webots/ no PATH"
export PATH=$PATH:$PWD/webots/