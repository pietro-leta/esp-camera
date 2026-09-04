#pragma once
#include <Arduino.h>
#include "esp_sleep.h"


namespace DeepSleep {
struct Config {
    // ======= Ajustes =======
    const gpio_num_t WAKE_PIN = GPIO_NUM_32;  // sinal do "comparador" com NPN
    const gpio_num_t B_SW     = GPIO_NUM_25;  // controle do barramento B (seu enable)

    const uint32_t AWAKE_MS = 10 * 1000;      // tempo acordado
    const uint32_t RELEASE_GUARD_MS = 300;    // debounce/anti-loop
    const uint32_t MAX_WAIT_RELEASE_MS = 5000;// evita travar se luz ficar presa

    // Se o seu barramento B liga com HIGH no GPIO25, deixe true.
    // Se for invertido, troque pra false.
    const bool B_ON_LEVEL_HIGH = true;
};

void setBarramentoB(bool on);

void waitWakeSignalReleaseIfNeeded();

void goDeepSleep();

void setupDeepSleep();

static Config cfg_;

}