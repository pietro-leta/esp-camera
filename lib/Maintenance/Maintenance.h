#pragma once
#include <Arduino.h>
#include <WiFi.h>

// forward declarations (evita include pesado no .h)
namespace SdLogger { class Logger; }
namespace LightSensors { class Manager; }

namespace Maintenance {

struct Config {
	// Wi-Fi / OTA
	const char* ssid = "";
	const char* pass = "";
	const char* otaHostname = "esp32";
	const char* otaPassword = "ota";

	// Pinos LEDs
	uint8_t ledWifi   = 2;
	uint8_t ledStatus = 4;

	// Botão físico de manutenção
	gpio_num_t maintBtn = GPIO_NUM_27;
	uint32_t btnDebounceMs = 30;

	// Pisca no boot
	uint16_t bootFlashOnMs  = 120;
	uint16_t bootFlashOffMs = 120;

	// Pisca LEDs em cada modo
	uint16_t blinkStatusMs     = 1000; // standby
	uint16_t blinkWifiWaitMs   = 800;  // conectando
	uint16_t blinkWifiReadyMs  = 100;  // conectado e pronto (OTA_READY)

	// Timeout de conexão Wi-Fi
	uint32_t wifiConnectTimeoutMs = 15000;

	// Verbose
	bool verbose = true;

	// ======== Novo: gatilho configurável ========
	enum class TriggerSource : uint8_t {
		PhysicalButton,     // só botão físico
		LightSensorButton,  // só "botão virtual" do sensor configurado
		Either              // botão físico OU sensor (default)
	};

	TriggerSource trigger = TriggerSource::Either;

	// Qual PIN do sensor deve disparar manutenção (default 35)
	// (usamos PIN, não índice, pra ficar robusto)
	int triggerLightPin = 35;
};

class Controller {
public:
	Controller(const Config& cfg, SdLogger::Logger* logger, LightSensors::Manager* sensors);
	void begin();
	void tick();

private:
	enum class Mode : uint8_t {
		STANDBY,
		WIFI_CONNECTING,
		OTA_READY,
		OTA_IN_PROGRESS
	};

	class WebServerWrapper; // definido no .cpp

	Config cfg_;
	SdLogger::Logger* logger_ = nullptr;
	LightSensors::Manager* sensors_ = nullptr;

	WebServerWrapper* web_ = nullptr;

	// estado Wi-Fi/HTTP
	bool wifiStarted_ = false;
	bool httpStarted_ = false;

	bool wifiLoggedConnected_ = false;
	bool wifiWasConnected_ = false;
	bool startedTimer_ = false;

	uint32_t wifiStartMs_ = 0;
	Mode mode_ = Mode::STANDBY;
	wl_status_t lastWiFiStatus_ = WL_IDLE_STATUS;

	// timers de LEDs e prints
	void bootFlashStatusLed();
	void updateLedLogic();

	// gatilhos
	bool maintButtonPressed();
	bool triggerLightPressed();
	bool maintenanceTriggered();

	// ações Wi-Fi/OTA/HTTP
	void wifiOtaOff();
	void startWifiAndOTA();
	void startHttpServer();

	// utils
	void logVerbose(const String& msg) const;
};

} // namespace Maintenance
