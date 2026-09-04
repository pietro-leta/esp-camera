#pragma once
#include <Arduino.h>

namespace SdLogger {

struct Config {
	// SD
	uint8_t sdCs = 5;
	uint32_t spiFreq = 16000000;

	// paths
	const char* wifiLogPath = "/wifi_log.csv";
	const char* sensorsLogPath = "/sensors_log.csv";

	bool verbose = true;
};

class Logger {
public:
	explicit Logger(const Config& cfg);

	bool begin();        // init SPI + SD, escreve headers
	bool ready() const;  // sd ok?

	void appendLine(const char* path, const String& line);

	void logWifiEvent(const char* event, const String& extra = "");
	void logSensorsRow(const String& csvLine);

	// info útil
	uint64_t cardSizeMB() const { return cardSizeMB_; }

private:
	Config cfg_;
	bool sdReady_ = false;
	uint64_t cardSizeMB_ = 0;

	void ensureWifiLogHeader();
	void ensureSensorsLogHeader();
	String sanitizeCsvField(String s);
};

} // namespace SdLogger
