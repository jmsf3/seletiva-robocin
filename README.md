# Seletiva-RobôCIn
Este repositório contém o projeto desenvolvido para o processo seletivo da equipe de drones do __[RobôCIn](https://robocin.com.br/)__, que abrange as áreas de __decisão__ e __movimentação__.

# Sobre
O projeto consiste no desenvolvimento de um sistema capaz de guiar um __drone__ __autônomo__, equipado com uma câmera __RGB__, por um trajeto na forma de um quadrado, cujos lados possuem cores distintas. O drone deve levantar voo no canto inferior esquerdo do quadrado e passar pelos quatro cantos apenas uma vez, antes de retornar à posição inicial para realizar o pouso.

# Configuração
A configuração do sistema desenvolvido é formada por dois nós __ROS__:
* __/robocin_pilot__ que é responsável pela movimentação do drone.
* __/camera_feed__ que é responsável pela captação e pelo processamento das imagens do drone.

# Execução
Após realizar o __[setup](https://bymateus.notion.site/Software-Setup-b3f9eecaa44946b0a59bfc81c0adb44e)__ do ambiente, para executar o sistema bastar fazer o seguinte:

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

# Por fim, abrir outro terminal na pasta "robocin_pilot" e rodar o "/robocin_pilot"
./robocin_pilot.py
```
