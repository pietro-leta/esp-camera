#include "Maintenance.h"

#include <ArduinoOTA.h>
#include <WebServer.h>

// Pode falhar em algumas builds/cores — vamos proteger
#if __has_include(<BluetoothSerial.h>)
  #include <BluetoothSerial.h>
  #define MAINT_HAS_BTSTOP 1
#else
  #define MAINT_HAS_BTSTOP 0
#endif

#include "../SdLogger/SdLogger.h"
#include "../LightSensors/LightSensors.h"

namespace Maintenance {

class Controller::WebServerWrapper {
public:
  WebServer http{80};
};

Controller::Controller(const Config& cfg, SdLogger::Logger* logger, LightSensors::Manager* sensors)
: cfg_(cfg), logger_(logger), sensors_(sensors) {}

void Controller::logVerbose(const String& msg) const {
  if (cfg_.verbose) Serial.println(msg);
}

void Controller::bootFlashStatusLed() {
  for (int i = 0; i < 3; i++) {
	digitalWrite(cfg_.ledStatus, HIGH);
	delay(cfg_.bootFlashOnMs);
	digitalWrite(cfg_.ledStatus, LOW);
	delay(cfg_.bootFlashOffMs);
  }
}

bool Controller::maintButtonPressed() {
  if (digitalRead((int)cfg_.maintBtn) == LOW) {
	delay(cfg_.btnDebounceMs);
	Serial.println("Botão Pressionado");
	return digitalRead((int)cfg_.maintBtn) == LOW;
  }
  return false;
}

bool Controller::triggerLightPressed() {
  if (!sensors_) return false;

  // Procura o sensor pelo PIN configurado (default 35)
  const uint8_t n = sensors_->count();
  for (uint8_t i = 0; i < n; i++) {
	const auto& st = sensors_->sensor(i);
	if ((int)st.pin == cfg_.triggerLightPin) {
	  return sensors_->button(i);
	}
  }
  return false; // não achou sensor com esse PIN
}

bool Controller::maintenanceTriggered() {
  switch (cfg_.trigger) {
	case Config::TriggerSource::PhysicalButton:
	  return maintButtonPressed();

	case Config::TriggerSource::LightSensorButton:
	  return triggerLightPressed();

	case Config::TriggerSource::Either:
	default:
	  return maintButtonPressed() || triggerLightPressed();
  }
}

void Controller::wifiOtaOff() {
  if (web_) web_->http.stop();
  httpStarted_ = false;

  ArduinoOTA.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

#if MAINT_HAS_BTSTOP
  btStop();
#endif

  wifiStarted_ = false;

  wifiLoggedConnected_ = false;
  wifiWasConnected_ = false;
  startedTimer_ = false;

  digitalWrite(cfg_.ledWifi, LOW);

  if (logger_) logger_->logWifiEvent("wifi_off");
}

void Controller::startHttpServer() {
  if (!web_) web_ = new WebServerWrapper();
  auto& http = web_->http;

  http.stop();

  http.on("/", [&]() {
	http.send(200, "text/html",
	  "<h2>ESP32 OK</h2>"
	  "<ul>"
	  "<li><a href='/status'>/status</a></li>"
	  "<li><a href='/sensor'>/sensor</a></li>"
	  "<li><a href='/SD_card'>/SD_card</a></li>"
	  "</ul>");
  });

  http.on("/status", [&]() {
	String body = "{";
	body += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + "\",";
	body += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
	body += "}";
	http.send(200, "application/json", body);
  });

  // JSON genérico para N sensores
  http.on("/sensor", [&]() {
	if (!sensors_) {
	  http.send(200, "application/json", "{\"n\":0,\"sensors\":[]}");
	  return;
	}

	auto snap = sensors_->get();

	String body;
	body.reserve(500);
	body += "{";
	body += "\"n\":" + String(snap.n) + ",";
	body += "\"triggerLightPin\":" + String(cfg_.triggerLightPin) + ",";
	body += "\"trigger\":\"";

	switch (cfg_.trigger) {
	  case Config::TriggerSource::PhysicalButton: body += "physical"; break;
	  case Config::TriggerSource::LightSensorButton: body += "light"; break;
	  case Config::TriggerSource::Either: default: body += "either"; break;
	}
	body += "\",";

	body += "\"sensors\":[";
	for (uint8_t i = 0; i < snap.n; i++) {
	  const auto& st = sensors_->sensor(i);
	  if (i) body += ",";

	  body += "{";
	  body += "\"idx\":" + String(i) + ",";
	  body += "\"pin\":" + String(st.pin) + ",";
	  body += "\"raw\":" + String(snap.raw[i]) + ",";
	  body += "\"filt\":" + String(snap.filt[i]) + ",";
	  body += "\"pct\":" + String(snap.pct[i]) + ",";
	  body += "\"btn\":" + String(snap.btn[i] ? 1 : 0) + ",";
	  body += "\"calDark\":" + String(snap.calDark[i]) + ",";
	  body += "\"calBright\":" + String(snap.calBright[i]);
	  body += "}";
	}
	body += "]}";

	http.send(200, "application/json", body);
  });

  http.on("/SD_card", [&]() {
	String body = "{";
	body += "\"sd\":\"" + String((logger_ && logger_->ready()) ? "ready" : "not_ready") + "\"";
	body += "}";
	http.send(200, "application/json", body);
  });

  http.begin();
}

void Controller::startWifiAndOTA() {
  if (wifiStarted_) return;

  startedTimer_ = false;
  wifiStartMs_ = 0;
  mode_ = Mode::WIFI_CONNECTING;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(cfg_.ssid, cfg_.pass);

  if (logger_) logger_->logWifiEvent("wifi_begin", cfg_.ssid);

  ArduinoOTA.setHostname(cfg_.otaHostname);
  ArduinoOTA.setPassword(cfg_.otaPassword);

  ArduinoOTA.onStart([&]() {
	mode_ = Mode::OTA_IN_PROGRESS;
	digitalWrite(cfg_.ledWifi, HIGH);
	if (logger_) logger_->logWifiEvent("ota_start");
  });

  ArduinoOTA.onEnd([&]() {
	if (logger_) logger_->logWifiEvent("ota_end");
	delay(200);
	ESP.restart();
  });

  ArduinoOTA.onError([&](ota_error_t error) {
	if (logger_) logger_->logWifiEvent("ota_error", String((unsigned)error));
	mode_ = (WiFi.status() == WL_CONNECTED) ? Mode::OTA_READY : Mode::WIFI_CONNECTING;
  });

  ArduinoOTA.begin();
  wifiStarted_ = true;
}

void Controller::updateLedLogic() {
  static uint32_t tLedWifi = 0;
  static uint32_t tLedStatus = 0;

  uint32_t now = millis();

  if (mode_ == Mode::STANDBY) {
	if (now - tLedStatus >= cfg_.blinkStatusMs) {
	  tLedStatus = now;
	  digitalWrite(cfg_.ledStatus, !digitalRead(cfg_.ledStatus));
	}
  } else {
	digitalWrite(cfg_.ledStatus, LOW);
  }

  if (mode_ == Mode::WIFI_CONNECTING) {
	if (now - tLedWifi >= cfg_.blinkWifiWaitMs) {
	  tLedWifi = now;
	  digitalWrite(cfg_.ledWifi, !digitalRead(cfg_.ledWifi));
	}
  } else if (mode_ == Mode::OTA_READY) {
	if (now - tLedWifi >= cfg_.blinkWifiReadyMs) {
	  tLedWifi = now;
	  digitalWrite(cfg_.ledWifi, !digitalRead(cfg_.ledWifi));
	}
  } else if (mode_ == Mode::OTA_IN_PROGRESS) {
	digitalWrite(cfg_.ledWifi, HIGH);
  } else {
	digitalWrite(cfg_.ledWifi, LOW);
  }
}

void Controller::begin() {
	pinMode(cfg_.ledWifi, OUTPUT);
	pinMode(cfg_.ledStatus, OUTPUT);
	digitalWrite(cfg_.ledWifi, LOW);
	digitalWrite(cfg_.ledStatus, LOW);

	pinMode((int)cfg_.maintBtn, INPUT_PULLUP);

	bootFlashStatusLed();
	wifiOtaOff();
	mode_ = Mode::STANDBY;

	if (cfg_.verbose) {
		Serial.println("[Maintenance] begin()");
		Serial.printf("[Maintenance] trigger=%d, maintBtn=%d, triggerLightPin=%d\n",
		(int)cfg_.trigger, (int)cfg_.maintBtn, cfg_.triggerLightPin);
	}
}

	void Controller::tick() {
	// LEDs sempre
	updateLedLogic();

	// standby: decide se liga wifi
	if (mode_ == Mode::STANDBY) {
		if (maintenanceTriggered()) startWifiAndOTA();
		return;
	}

	// modo manutenção
	if (wifiStarted_) ArduinoOTA.handle();
	if (httpStarted_ && web_) web_->http.handleClient();

	if (!startedTimer_) {
		startedTimer_ = true;
		wifiStartMs_ = millis();
	}

	wl_status_t st = WiFi.status();
	if (st != lastWiFiStatus_) {
		lastWiFiStatus_ = st;
		if (logger_) logger_->logWifiEvent("wifi_status", String((int)st));
	}

  	if (st == WL_CONNECTED) {
	wifiWasConnected_ = true;

	if (!wifiLoggedConnected_) {
	  	wifiLoggedConnected_ = true;
	  	if (logger_) logger_->logWifiEvent(
		"wifi_connected",
		"ip=" + WiFi.localIP().toString() + ";rssi=" + String(WiFi.RSSI()));
	}

	mode_ = (mode_ == Mode::OTA_IN_PROGRESS) ? Mode::OTA_IN_PROGRESS : Mode::OTA_READY;

	if (!httpStarted_) {
		startHttpServer();
	  	httpStarted_ = true;
	  	if (logger_) logger_->logWifiEvent("http_started");
	}

  	} else {
	if (wifiWasConnected_) {
		wifiWasConnected_ = false;
		wifiLoggedConnected_ = false;
		if (logger_) logger_->logWifiEvent("wifi_disconnected");
	}

	mode_ = Mode::WIFI_CONNECTING;

	if (millis() - wifiStartMs_ >= cfg_.wifiConnectTimeoutMs) {
		if (logger_) logger_->logWifiEvent("wifi_timeout", String(cfg_.wifiConnectTimeoutMs));
		wifiOtaOff();
	  	mode_ = Mode::STANDBY;
	}
  }
}

} // namespace Maintenance