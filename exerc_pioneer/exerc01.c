/**
 * @file exerc01.c
 * @brief Desenvolver um programa em C para controlar um robô Pioneer 3-DX
 *        no simulador Webots, fazendo com que ele percorra uma trajetória
 *        em formato de quadrado.
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
    link_t motor = ufr_publisher("@new webots @topic /cmd_vel @time 1000ms");

    // Loop Principal (ctrl-c para sair)
    while ( ufr_loop_ok() ) {
        // envia velocidade linear (m/s) e velocidade angular (radianos/s)
        ufr_put(&motor, "ff", 1.0, M_PI/4 );
    }

    // Fim
    ufr_close(&motor);
    return 0;
}