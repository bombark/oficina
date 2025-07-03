source /etc/os-release

# Linux Mint - Debian 12
if [[ $ID == "linuxmint" ]]; then
    echo "adicionado ./lib-debian12 no LD_LIBRARY_PATH"
    export LD_LIBRARY_PATH=$PWD/lib-debian12/
    
# Ubuntu 22
elif [[ $ID == "ubuntu" ]]; then
    echo "adicionado ./lib-ubuntu22 no LD_LIBRARY_PATH"
    export LD_LIBRARY_PATH=$PWD/lib-ubuntu22/

# Erro
else
    echo "Linux nao identificado"
fi
