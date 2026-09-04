#include "LightSensors.h"

namespace LightSensors {

Manager::Manager(const Config& cfg) : cfg_(cfg) {}

int Manager::clampi(int x, int a, int b) {
	return (x < a) ? a : (x > b) ? b : x;
}

int Manager::addSensor(uint8_t pin, bool asButton) {
	if (n_ >= MAX_SENSORS) return -1;
	s_[n_] = SensorState(pin, asButton);
	return n_++;
}

void Manager::sensorInit(SensorState& s) {
	if (s.pin == 255) return;
	analogSetPinAttenuation(s.pin, cfg_.attenuation);

	s.filtered = analogRead(s.pin);
	s.lastFilt = (int)(s.filtered + 0.5f);

	// defaults sensatos, mas você calibra via serial
	s.calDark   = s.lastFilt;
	s.calBright = s.lastFilt + 100; // evita denom=0 no começo
}

void Manager::sensorUpdate(SensorState& s) {
	s.lastRaw = analogRead(s.pin);
	s.filtered = cfg_.alpha * s.lastRaw + (1.0f - cfg_.alpha) * s.filtered;
	s.lastFilt = (int)(s.filtered + 0.5f);

	int denom = max(1, s.calBright - s.calDark);
	int pct = ((s.lastFilt - s.calDark) * 100) / denom;
	s.lastPct = clampi(pct, 0, 100);
}

void Manager::updateVirtualButton(SensorState& s) {
	if (!s.hasButton) return;

	if (!s.btnState && s.lastPct >= cfg_.btnOnPct) s.btnState = true;
	else if (s.btnState && s.lastPct <= cfg_.btnOffPct) s.btnState = false;
}

void Manager::printAllCal() const {
	for (uint8_t i = 0; i < n_; i++) {
		Serial.printf("[%u] pin=%u calDark=%d calBright=%d btn=%d\n",
			i, s_[i].pin, s_[i].calDark, s_[i].calBright, s_[i].btnState ? 1 : 0);
	}
}

void Manager::handleSerialCommands() {
	while (Serial.available()) {
		String cmd = Serial.readStringUntil('\n');
		cmd.trim();
		if (cmd.length() == 0) continue;

		if (cmd == "p") {
			printAllCal();
			continue;
		}

		// Formatos: d<idx> ou b<idx>  ex: d0 b2
		char op = cmd.charAt(0);
		int idx = cmd.substring(1).toInt();

		if (idx < 0 || idx >= (int)n_) {
			Serial.println("Idx invalido. Use p para listar. Ex: d0 b2");
			continue;
		}

		if (op == 'd') {
			s_[idx].calDark = s_[idx].lastFilt;
			Serial.printf("[%d] pin=%u ESCURO=%d\n", idx, s_[idx].pin, s_[idx].calDark);
		} else if (op == 'b') {
			s_[idx].calBright = s_[idx].lastFilt;
			Serial.printf("[%d] pin=%u CLARO=%d\n", idx, s_[idx].pin, s_[idx].calBright);
		} else {
			Serial.println("Comandos: d<idx> b<idx> p   (ex: d0, b2, p)");
		}
	}
}

void Manager::begin() {
	analogReadResolution(12);

	for (uint8_t i = 0; i < n_; i++) {
		sensorInit(s_[i]);
	}

	uint32_t now = millis();
	nextSample_ = now + cfg_.sampleMs;
	nextPrint_  = now + cfg_.printMs;

	if (cfg_.verbose) {
		Serial.println("Comandos sensores: d<idx> b<idx> p   (ex: d0, b2, p)");
		printAllCal();
	}
}

void Manager::tick() {
	const uint32_t now = millis();

	if ((int32_t)(now - nextSample_) >= 0) {
		nextSample_ += cfg_.sampleMs;

		for (uint8_t i = 0; i < n_; i++) {
			sensorUpdate(s_[i]);
			updateVirtualButton(s_[i]);
		}
	}

	if ((int32_t)(now - nextPrint_) >= 0) {
		nextPrint_ += cfg_.printMs;
		printReady_ = true;

		if (cfg_.verbose) {
			for (uint8_t i = 0; i < n_; i++) {
				Serial.printf("[%u] pin=%u raw=%d filt=%d pct=%d btn=%d  ",
					i, s_[i].pin, s_[i].lastRaw, s_[i].lastFilt, s_[i].lastPct, s_[i].btnState ? 1 : 0);
			}
			Serial.println();
		}
	}

	handleSerialCommands();
}

bool Manager::consumePrintTick() {
	if (!printReady_) return false;
	printReady_ = false;
	return true;
}

String Manager::makeCsvHeader() const {
	// Ex: ms,s0_raw,s0_filt,s0_pct,s0_btn,s1_raw,...
	String h;
	h.reserve(200);
	h += "ms";
	for (uint8_t i = 0; i < n_; i++) {
		h += ",s"; h += i; h += "_raw";
		h += ",s"; h += i; h += "_filt";
		h += ",s"; h += i; h += "_pct";
		h += ",s"; h += i; h += "_btn";
		h += ",s"; h += i; h += "_calDark";
		h += ",s"; h += i; h += "_calBright";
	}
	return h;
}

String Manager::makeCsvRow() const {
	String line;
	line.reserve(250);

	line += String(millis());
	for (uint8_t i = 0; i < n_; i++) {
		line += ",";
		line += String(s_[i].lastRaw);
		line += ",";
		line += String(s_[i].lastFilt);
		line += ",";
		line += String(s_[i].lastPct);
		line += ",";
		line += String(s_[i].btnState ? 1 : 0);
		line += ",";
		line += String(s_[i].calDark);
		line += ",";
		line += String(s_[i].calBright);
	}

	return line;
}

Snapshot Manager::get() const {
	Snapshot snap;
	snap.n = n_;
	for (uint8_t i = 0; i < n_ && i < MAX_SENSORS; i++) {
		snap.raw[i] = s_[i].lastRaw;
		snap.filt[i] = s_[i].lastFilt;
		snap.pct[i] = s_[i].lastPct;
		snap.btn[i] = s_[i].btnState ? 1 : 0;
		snap.calDark[i] = s_[i].calDark;
		snap.calBright[i] = s_[i].calBright;
	}
	return snap;
}

} // namespace LightSensors