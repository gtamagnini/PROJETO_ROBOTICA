// CENTRO UNIVERSITARIO FEI - CC7711 - PROJETO ROBOTICA

// Integrantes: 
// Gabryel Lourenço Maciel de Morais 
// Giovanna Borges Tamagnini
// Thiago Ayres Kimura 

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define TIME_STEP 256
#define MAX_SPEED 6.28 // Velocidade máxima permitida para o E-Puck
#define BASE_SPEED 1.0 // Velocidade base do robô no intervalo [-1.0, 1.0]
#define THRESHOLD_LIGHT_BOX_DISTANCE 400
#define ROTATION_SPEED 2.0 // Velocidade de rotação no intervalo [-1.0, 1.0]
#define DISTANCE_THRESHOLD 800

double limitar_velocidade(double velocidade) {
    if (velocidade > MAX_SPEED) return MAX_SPEED;
    if (velocidade < -MAX_SPEED) return -MAX_SPEED;
    return velocidade;
}

int main() {
    wb_robot_init();

    // Inicialização dos dispositivos do robô
    WbDeviceTag motor_esq = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_dir = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(motor_esq, INFINITY);
    wb_motor_set_position(motor_dir, INFINITY);

    // Configuração dos sensores de proximidade
    WbDeviceTag sensores[8];
    for (int i = 0; i < 8; i++) {
        char sensor_name[6];
        sprintf(sensor_name, "ps%d", i);
        sensores[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(sensores[i], TIME_STEP);
    }

    // Configuração dos LEDs
    WbDeviceTag leds[10];
    for (int i = 0; i < 10; i++) {
        char led_name[6];
        sprintf(led_name, "led%d", i);
        leds[i] = wb_robot_get_device(led_name);
        wb_led_set(leds[i], 0);
    }

    // Supervisão do objeto alvo
    WbNodeRef objeto_alvo = wb_supervisor_node_get_from_def("WOODEN_BOX");
    const double *posicao_inicial = wb_supervisor_node_get_position(objeto_alvo);
    double posicao_referencia[3] = {posicao_inicial[0], posicao_inicial[1], posicao_inicial[2]};

    double acelerador_direito = 1.0, acelerador_esquerdo = 1.0;
    int found_light_box = 0;

    while (wb_robot_step(TIME_STEP) != -1) {
        if (found_light_box) break;

        double leitura_sensores[8];
        char buffer[256] = {0};

        // Coleta de dados dos sensores
        for (int i = 0; i < 8; i++) {
            leitura_sensores[i] = wb_distance_sensor_get_value(sensores[i]) - 60;
            sprintf(buffer + strlen(buffer), "Sensor %d: %.2f ", i, leitura_sensores[i]);
        }
        printf("%s\n", buffer);

        // Verifica a posição do objeto alvo
        const double *posicao_atual = wb_supervisor_node_get_position(objeto_alvo);
        if (fabs(posicao_referencia[0] - posicao_atual[0]) > 0.001 ||
            fabs(posicao_referencia[1] - posicao_atual[1]) > 0.001 ||
            fabs(posicao_referencia[2] - posicao_atual[2]) > 0.001) {

            // Parar o robô e acender os LEDs
            acelerador_direito = 0;
            acelerador_esquerdo = 0;
            wb_motor_set_velocity(motor_esq, 0);
            wb_motor_set_velocity(motor_dir, 0);

            for (int i = 0; i < 10; i++) {
                wb_led_set(leds[i], 1);
            }

            printf("Caixa leve encontrada! Parando o robô.\n");
            found_light_box = 1;
            break;
        }

        // Lógica de movimento com base nos sensores
        if (leitura_sensores[0] > 10) {
            acelerador_direito = -1.0; // Gira para evitar o obstáculo
            acelerador_esquerdo = 1.0;
        } else {
            acelerador_direito = 1.0; // Continua em linha reta
            acelerador_esquerdo = 1.0;
        }

        // Aplicar velocidades ajustadas aos motores
        wb_motor_set_velocity(motor_esq, MAX_SPEED * acelerador_esquerdo);
        wb_motor_set_velocity(motor_dir, MAX_SPEED * acelerador_direito);
    }

    wb_robot_cleanup();
    return 0;
}
