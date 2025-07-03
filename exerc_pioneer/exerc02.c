/**
 * @file exerc02.c
 * @brief Desenvolver um programa em C para controlar um robô Pioneer 3-DX
 *        no simulador Webots, fazendo com que ele percorra uma trajetória
 *        em formato de quadrado. Contudo utilizando a odometria para ser
 *        mais preciso
 */

// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <stdio.h>
#include <math.h>

// ============================================================================
//  Main
// ============================================================================

int main() {
    // Abre o link para o controle dos motores
    link_t motor = ufr_publisher("@new webots @topic /cmd_vel @time 200ms");

    // Abre o link para a leitura da odometria
    link_t odom = ufr_subscriber("@new webots @topic /pose");
    float x,y,th_rad;

    // Loop Principal (ctrl-c para sair)
    while ( ufr_loop_ok() ) {
        // le a odometria x(m), y(m), theta(rad)
        ufr_get(&odom, "^fff", &x, &y, &th_rad);
        const double th_graus = th_rad * 180.0 / M_PI;
        printf("%f %f %f\n", x,y,th_graus);

        // envia velocidade linear (m/s) e velocidade angular (radianos/s)
        ufr_put(&motor, "ff\n", 0.0, -1.0);
    }

    // Fim
    ufr_close(&odom);
    ufr_close(&motor);
    return 0;
}