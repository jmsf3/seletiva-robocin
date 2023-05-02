# Seletiva-RobôCIn
Este repositório contém o projeto desenvolvido para o processo seletivo da equipe de drones do ___[RobôCIn](https://robocin.com.br/)___, que abrange as áreas de ___decisão___ e ___movimentação___.

# Sobre
O projeto consiste em um sistema de ___decisão___ e ___movimentação___ que é capaz de guiar um ___drone autônomo___, equipado com uma câmera ___RGB___ que aponta para o solo, através de um trajeto na forma de um quadrado com os lados coloridos em cores distintas. O drone passa por cada um dos cantos do quadrado uma única vez antes de retornar à posição inicial, onde deve realizar o seu pouso.

# Configuração
A configuração do sistema é formada por dois nós ___ROS___:
* ___/pilot___ que é responsável pela movimentação do drone.
* ___/camera_feed___ que é responsável pelo processamento da imagem da câmera do drone.

# Execução
Após realizar o ___[setup](https://bymateus.notion.site/Software-Setup-b3f9eecaa44946b0a59bfc81c0adb44e)___ do ambiente, para executar o sistema bastar fazer o seguinte:

```
# Abrir o QGroundControl

# Abrir o terminal na pasta "quadrado_seletiva_gazebo_world" e rodar o Gazebo no mapa do desafio
gazebo --verbose quadrado_seletiva_gazebo.world

# Abrir outro terminal e rodar o Ardupilot SITL
sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy

# Abrir outro terminal e rodar o MAVProxy
mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 --out 127.0.0.1:14551

# Abrir outro terminal na pasta "camera_feed" e rodar o "/camera_feed"
./camera_feed.py

# Por fim, abrir outro terminal na pasta "robocin_pilot" e rodar o "/pilot"
./robocin_pilot.py
```
