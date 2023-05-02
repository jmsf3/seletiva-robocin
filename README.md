# Seletiva-RobôCIn
Este repositório contém o projeto desenvolvido para o processo seletivo da equipe de drones do ___[RobôCIn](https://robocin.com.br/)___, que abrange as áreas de ___decisão___ e ___movimentação___.

# Sobre
O projeto consiste em um sistema de ___decisão___ e ___movimentação___ que é capaz de guiar um ___drone autônomo___, equipado com uma câmera ___RGB___ que aponta para o solo, através de um trajeto na forma de um quadrado com os lados coloridos em cores distintas. O drone passa por cada um dos cantos do quadrado uma única vez antes de retornar à posição inicial, onde deve realizar o seu pouso.

# Configuração
A configuração do sistema é formada por dois nós ___ROS___:
* ___/pilot___ que é responsável pela movimentação do drone.
* ___/camera_feed___ que é responsável pelo processamento da imagem da câmera do drone.
