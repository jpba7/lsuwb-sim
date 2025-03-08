#!/bin/bash

# Define permissões de execução localmente
chmod +x src/*.py

# Remove container antigo se existir
docker rm -f lsuwb_simulator 2>/dev/null

# Executa novo container com volume montado
docker run -it --rm \
    --network host \
    --add-host=host.docker.internal:host-gateway \
    --name lsuwb_simulator \
    -v $(pwd)/src:/lsuwb_ws/src/lsuwb/src \
    lsuwb-simulator