#!/bin/bash

# Define permissões de execução localmente
chmod +x src/*.py

# Remove container antigo se existir
docker rm -f lsuwb_simulator 2>/dev/null

# Cria a nova imagem atualizada
docker build -t lsuwb-simulator .

# Executa novo container com volume montado
docker run -it \
    --network host \
    --add-host=host.docker.internal:host-gateway \
    --name lsuwb_simulator \
    -v $(pwd)/src:/lsuwb_ws/src/lsuwb/src \
    --restart unless-stopped \
    lsuwb-simulator