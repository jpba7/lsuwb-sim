# **LSUWB - Pacote ROS para Localização UWB**

O **LSUWB** é um pacote ROS desenvolvido para sistemas de localização utilizando tecnologia **UWB (Ultra-Wideband)**. Ele lê dados de um dispositivo serial, publica as informações no formato **`geometry_msgs/PoseStamped`** e facilita a integração com sistemas de navegação no ROS.

## **Estrutura da Mensagem Publicada**

O pacote publica mensagens do tipo **`PoseStamped`** em tópicos no formato `/lsuwb/<ID>` para cada TAG detectada.

### **Formato das Mensagens (`geometry_msgs/PoseStamped`):**
- **`header`**:
  - `stamp`: Timestamp da mensagem.
  - `frame_id`: Nome do frame de referência (ex.: `"map"`).
- **`pose`**:
  - `position`: Posição da TAG no espaço (x, y, z).
  - `orientation`: Orientação no espaço (fixada em `Quaternion(0, 0, 0, 1)` por padrão).

**Exemplo de mensagem publicada no tópico `/lsuwb/Test1`:**
```plaintext
header: 
  seq: 12
  stamp: 
    secs: 1692370654
    nsecs: 789452000
  frame_id: "map"
pose: 
  position: 
    x: 2.5
    y: 3.7
    z: 1.2
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
---
```

## **Instalação**

1. Certifique-se de que o ROS Noetic esteja instalado no seu sistema:  
    - Siga o guia oficial de instalação do ROS Noetic: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. Clone o repositório do pacote no seu workspace Catkin:
  
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/MrMaximo/lsuwb.
  ```
3. Compile o workspace:
  
  ```bash
  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash
  ```
---

## **Uso**

1. Executar o ROS Master

  Antes de executar qualquer nó, inicie o ROS Master com:
  
  ```bash
  roscore
  ```

2. Rodar o Pacote
  
  2.1 Com Arquivo de Launch

  Execute o nó principal e configure o dispositivo serial usando o arquivo de lançamento:
  
  ```bash
  roslaunch lsuwb lsuwb.launch
  ```

  2.2 Executando Diretamente o Nó

  Você também pode executar diretamente o script Python:
  
  ```bash
  rosrun lsuwb main.py
  ```
---

## **Configurações**

1. Parâmetros do Arquivo de Launch

No arquivo lsuwb.launch, você pode configurar:
- Porta Serial:
  - Valor padrão: /dev/ttyACM0
        
  ```xml
  <param name="serial_port" value="/dev/ttyACM0" />
  ```
- Baudrate:
  - Valor padrão: 115200
      
  ```xml
  <param name="baudrate" value="115200" />
  ```
2. Frame de Referência

  O frame de referência padrão para o PoseStamped é `"map"`. Se precisar alterar:
  
    ``` python
    msg.header.frame_id = "odom"
    ```
---

## **Visualizando as Mensagens**

1. Liste os tópicos publicados:
   
  ```bash
  rostopic list
  ```

  Você verá tópicos no formato /lsuwb/<ID> (ex.: /lsuwb/XPTO).

2. Monitore um tópico específico:
   
  ```bash
    rostopic echo /lsuwb/XPTO
  ```
---

## **Estrutura do Pacote**

- A estrutura do pacote lsuwb é a seguinte:
  
  ```bash
  lsuwb/
  ├── CMakeLists.txt          # Configuração de build do ROS
  ├── package.xml             # Configuração do pacote
  ├── launch/
  │   └── lsuwb.launch        # Arquivo de lançamento
  ├── msg/
  │   └── Tag.msg             # (Opcional) Arquivo de mensagem personalizada
  ├── src/
  │   ├── main.py             # Script principal
  │   └── tag.py              # Classe Tag
  ```

  ---

##  **Contribuição**

1. Faça um fork do projeto.
2. Crie um branch para suas alterações:
   
  ```bash
  git checkout -b minha-alteracao
  ```

3. Envie um Pull Request.

---
