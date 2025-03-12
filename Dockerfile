# Usa uma imagem base do ROS
FROM osrf/ros:noetic-desktop-full

# Define variáveis de ambiente
ENV ROS_DISTRO=noetic
ENV LSUWB_WS=/lsuwb_ws
ENV DEBIAN_FRONTEND=noninteractive

# Instala pacotes necessários
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    ros-${ROS_DISTRO}-rosbridge-server \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Inicializa o rosdep
RUN rosdep update

# Cria e configura o workspace
RUN mkdir -p $LSUWB_WS/src
WORKDIR $LSUWB_WS/src

# Copia os arquivos do projeto
COPY . lsuwb/

# Retorna ao diretório do workspace
WORKDIR $LSUWB_WS

# Instala dependências do ROS
RUN rosdep install --from-paths src --ignore-src -r -y

# Instala dependências Python adicionais
RUN pip3 install requests python-dotenv

# Compila o workspace
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

# Configura o ambiente ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $LSUWB_WS/devel/setup.bash" >> ~/.bashrc &&\
    echo "export ROS_MASTER_URI=http://192.168.0.100:11311" >> ~/.bashrc


# Define permissões de execução para os scripts Python
RUN chmod +x $LSUWB_WS/src/lsuwb/src/*.py

# Cria script de entrada
RUN echo '#!/bin/bash\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
source $LSUWB_WS/devel/setup.bash\n\
chmod +x $LSUWB_WS/src/lsuwb/src/*.py\n\
\n\
while true; do\n\
    sleep 1\n\
done\n\
' > /entrypoint.sh && chmod +x /entrypoint.sh

# Define o ponto de entrada
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]