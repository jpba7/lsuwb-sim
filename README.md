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
