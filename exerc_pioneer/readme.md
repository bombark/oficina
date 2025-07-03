# Exercicios para o Pioneer




## 0. Inicializar o WeBots no cenario do Pioneer

```
webots pioneer3dx.wbt
```

Se voce pausar o simulador, travará o programa

## 1. Robô Pioneiro se movimentando um Quadrado no Webots

Desenvolver um programa em C para controlar um robô Pioneer 3-DX no simulador Webots, fazendo com que ele percorra uma trajetória em formato de quadrado.

Cenário:

Você tem um robô Pioneer 3-DX em um ambieWnte simples no Webots. Seu desafio é programar o robô para se mover em um caminho quadrado, com cada lado tendo um comprimento definido.

Requisitos:

- Movimentação: O robô deve se mover para frente por um determinado tempo para completar um lado do quadrado.
- Curva: Após cada lado, o robô deve girar 90 graus para a direita para iniciar o próximo lado.
- Repetição: Os passos 1 e 2 devem ser repetidos quatro vezes para completar o quadrado.
- Parada: O robô deve parar ao completar o quadrado.

Compilaçao:

```
gcc exerc01.c -o exerc01 -lufr -I../../include
```

Execuçao:

```
./exerc01
```


## 2. Leitura da Posição e do Lidar



Compilaçao:

```
gcc exerc01.c -o exerc01 -lufr -I../../include
```

Execuçao:

```
./exerc01
```


## 3. Movimentar o robo até proximo de um ponto

Navegação por Odometria para um Ponto Alvo com Robô Pioneiro no Webots

Objetivo: 

Desenvolver um programa em C para controlar um robô Pioneer 3-DX no simulador Webots, fazendo com que ele se mova de sua posição inicial (origem 0,0) até um ponto predefinido no mapa, utilizando odometria para estimar sua posição atual.

Cenário:

Seu robô Pioneer 3-DX é ligado na posição (0,0) de um ambiente no Webots. Você precisa programá-lo para navegar até um ponto de destino específico, como (X, Y), utilizando a informação dos encoders das rodas para calcular a odometria e, assim, saber a posição aproximada do robô no ambiente.

Requisitos:

    Ponto de Destino: Defina um ponto de destino (por exemplo, target_x = 1.0 metro, target_y = 1.0 metro) no seu código.

    Odometria:

        Monitore as leituras dos encoders das rodas (direita e esquerda).

        Implemente um algoritmo de odometria para calcular a posição (x, y) e a orientação (theta) estimadas do robô em cada passo de tempo. Lembre-se que o Webots opera com coordenadas x para frente e y para a esquerda (dependendo da orientação inicial do robô no mundo, mas assumindo uma configuração padrão, o eixo X pode ser o "frente" e o Y o "lado").

    Controle de Movimento:

        O robô deve primeiro girar para apontar na direção do ponto alvo.

        Em seguida, deve se mover para frente até se aproximar do ponto alvo.

        Um controle simples de feedback (por exemplo, baseado na distância restante e no ângulo para o alvo) pode ser usado para ajustar o curso e a velocidade.

    Critério de Parada: O robô deve parar quando estiver dentro de uma pequena margem de erro (e.g., 0.1 metros) do ponto de destino.

    Simulação em Tempo Real: O código deve funcionar no loop de simulação do Webots.

Configuração do Webots:

    Mantenha um mundo simples com um robô Pioneer 3-DX.

    Certifique-se de que o robô está posicionado na origem (0,0,0) do mundo quando a simulação começa (ou ajuste a origem em seu cálculo de odometria).

    No controlador do Pioneer, defina o tipo de linguagem como C.

Fórmulas Essenciais para Odometria (Revisão):

Assumindo que delta_L e delta_R são as distâncias percorridas pelas rodas esquerda e direita, B é a distância entre as rodas (distância entre eixos), e R é o raio das rodas:

    Distâncias das rodas:
        delta_L = current_encoder_left - previous_encoder_left
        delta_R = current_encoder_right - previous_encoder_right

        Multiplique pelo raio da roda para obter a distância linear percorrida.

    Deslocamento linear médio:

        delta_D = (delta_L + delta_R) / 2.0

    Mudança angular:

        delta_theta = (delta_R - delta_L) / B

    Atualização da posição (x, y) e orientação (theta):

        new_theta = old_theta + delta_theta
        new_x = old_x + delta_D * cos(new_theta)
        new_y = old_y + delta_D * sin(new_theta)


Compilaçao:
```
gcc exerc03.c -o exerc03 -lufr -I../../include
```

Execuçao:
```
./exerc03
```


## 4. 