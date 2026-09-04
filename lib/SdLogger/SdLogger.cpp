#include "SdLogger.h"
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>

namespace SdLogger {

Logger::Logger(const Config& cfg) : cfg_(cfg) {}

bool Logger::begin() {
	// Usa os pinos default do ESP32 (VSPI) a não ser que você precise fixar.
	SPI.begin();

	sdReady_ = SD.begin(cfg_.sdCs, SPI, cfg_.spiFreq);
	if (!sdReady_) {
		if (cfg_.verbose) Serial.println("[SD] Falha ao iniciar (seguindo sem SD).");
		return false;
	}

	if (SD.cardType() == CARD_NONE) {
		sdReady_ = false;
		if (cfg_.verbose) Serial.println("[SD] Nenhum cartão detectado (seguindo sem SD).");
		return false;
	}

	cardSizeMB_ = SD.cardSize() / (1024ULL * 1024ULL);
	if (cfg_.verbose) Serial.printf("[SD] Cartão OK. Tamanho: %llu MB\n", cardSizeMB_);

	ensureWifiLogHeader();
	ensureSensorsLogHeader();
	logWifiEvent("boot");

	return true;
}

bool Logger::ready() const { return sdReady_; }

String Logger::sanitizeCsvField(String s) {
	s.replace("\n", " ");
	s.replace("\r", " ");
	s.replace(",", ";");
	return s;
}

void Logger::appendLine(const char* path, const String& line) {
	if (!sdReady_) return;

	File f = SD.open(path, FILE_APPEND);
	if (!f) {
		sdReady_ = false;
		Serial.println("[SD] Falha ao abrir arquivo para append. Marcando sdReady=false.");
		return;
	}
	f.println(line);
	f.close();
}

void Logger::ensureWifiLogHeader() {
	if (!sdReady_) return;
	if (SD.exists(cfg_.wifiLogPath)) return;

	File f = SD.open(cfg_.wifiLogPath, FILE_WRITE);
	if (!f) { sdReady_ = false; Serial.println("[SD] Falha ao criar wifi_log.csv."); return; }
	f.println("ms,event,ssid,ip,rssi,extra");
	f.close();
}

void Logger::ensureSensorsLogHeader() {
	if (!sdReady_) return;
	if (SD.exists(cfg_.sensorsLogPath)) return;

	File f = SD.open(cfg_.sensorsLogPath, FILE_WRITE);
	if (!f) { sdReady_ = false; Serial.println("[SD] Falha ao criar sensors_log.csv."); return; }

	f.println("ms,raw34,filt34,pct34,raw35,filt35,pct35,btn35,calD34,calB34,calD35,calB35");
	f.close();
}

void Logger::logWifiEvent(const char* event, const String& extra) {
	if (!sdReady_) return;

	String ssid = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : "";
	String ip   = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "";
	int rssi    = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

	String safeExtra = sanitizeCsvField(extra);

	String line;
	line.reserve(220);
	line += String(millis()); line += ",";
	line += event;            line += ",";
	line += ssid;             line += ",";
	line += ip;               line += ",";
	line += String(rssi);     line += ",";
	line += safeExtra;

	appendLine(cfg_.wifiLogPath, line);
}

void Logger::logSensorsRow(const String& csvLine) {
	appendLine(cfg_.sensorsLogPath, csvLine);
}

} // namespace SdLogger
