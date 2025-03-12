#!/bin/bash

# Define permissões de execução localmente
chmod +x src/*.py

# Executa novo container com volume montado
docker run -it \
    --network host \
    --add-host=host.docker.internal:host-gateway \
    --name lsuwb_simulator \
    -v $(pwd)/src:/lsuwb_ws/src/lsuwb/src \
    --restart unless-stopped \
    lsuwb-simulator