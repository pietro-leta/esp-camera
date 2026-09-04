#include "DeepSleep.h"

#include <Arduino.h>
#include "esp_sleep.h"

namespace DeepSleep {

void setBarramentoB(bool on) {
	bool level = cfg_.B_ON_LEVEL_HIGH ? on : !on;
	digitalWrite((int)cfg_.B_SW, level ? HIGH : LOW);
}

void waitWakeSignalReleaseIfNeeded() {
	// Seu evento de wake é: GPIO32 = LOW (transistor puxou pro GND)
	// Se ainda estiver LOW logo após acordar, esperamos soltar (voltar HIGH)
	// para não ficar num loop acorda->dorme->acorda enquanto a luz permanece.
	if (digitalRead((int)cfg_.WAKE_PIN) == LOW) {
		Serial.println("[GUARD] WAKE ainda LOW (luz ativa). Aguardando voltar HIGH...");
		uint32_t t0 = millis();
		while (digitalRead((int)cfg_.WAKE_PIN) == LOW && (millis() - t0) < cfg_.MAX_WAIT_RELEASE_MS) {
			delay(10);
		}
		delay(cfg_.RELEASE_GUARD_MS);
	}
}

// esse é o "loop" do deep sleep
void goDeepSleep() {
	Serial.println("[SLEEP] Desligando barramento B e entrando em deep sleep...");
	Serial.flush();

	setBarramentoB(false);

	// EXT0: acorda por nível em um pino RTC (GPIO32 é RTC)
	// Acorda quando WAKE_PIN estiver LOW
	esp_sleep_enable_ext0_wakeup(cfg_.WAKE_PIN, 0);

	Serial.println("[SLEEP] Deep sleep iniciado. (Wake quando GPIO32 ficar LOW)");
	Serial.flush();

	esp_deep_sleep_start();
}

void setupDeepSleep() {
	// Configura pinos
	pinMode((int)cfg_.B_SW, OUTPUT);
	setBarramentoB(true); // liga B ao acordar/boot

	// Pull-up do WAKE é externo, então INPUT puro
	pinMode((int)cfg_.WAKE_PIN, INPUT);

	// Log de causa do wake
	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	Serial.print("[BOOT] Wake cause: ");
	if (cause == ESP_SLEEP_WAKEUP_EXT0) Serial.println("EXT0 (GPIO32)");
	else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) Serial.println("POWER-ON/RESET");
	else Serial.println("OTHER");

	// Anti-loop se luz ficou ligada
	waitWakeSignalReleaseIfNeeded();
}

}