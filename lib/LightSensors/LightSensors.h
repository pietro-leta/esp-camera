#pragma once
#include <Arduino.h>

namespace LightSensors {

struct Config {
	float alpha = 0.20f;          // filtro IIR
	uint32_t sampleMs = 100;      // período de amostragem
	uint32_t printMs  = 1000;     // período de log
	bool verbose = false;

	// histerese do botão virtual (em %)
	int btnOnPct  = 90;
	int btnOffPct = 80;

	// Atenuação do ADC (ESP32): ADC_11db é comum p/ faixa maior
	adc_attenuation_t attenuation = ADC_11db;
};

// Snapshot genérico: suporta até MAX_SENSORS
static constexpr uint8_t MAX_SENSORS = 8;

struct Snapshot {
	uint8_t n = 0;
	int raw[MAX_SENSORS]   = {0};
	int filt[MAX_SENSORS]  = {0};
	int pct[MAX_SENSORS]   = {0};
	uint8_t btn[MAX_SENSORS] = {0};
	int calDark[MAX_SENSORS]   = {0};
	int calBright[MAX_SENSORS] = {0};
};

struct SensorState {
	uint8_t pin = 255;

	// leituras
	int lastRaw  = 0; // leitura direta do pino
	float filtered = 0.0f; // filtra le os calDark e calBright e alpha
	int lastFilt = 0;
	int lastPct  = 0;

	// calibração
	int calDark   = 20;
	int calBright = 2000;

	// botão virtual
	bool hasButton = true;
	bool btnState  = false;

	SensorState() = default;
	explicit SensorState(uint8_t p, bool asButton = true) : pin(p), hasButton(asButton) {}
};

class Manager {
public:
	explicit Manager(const Config& cfg);

	// Adiciona sensor (chame no setup, antes de begin)
	// Retorna índice do sensor (0..n-1) ou -1 se lotado
	int addSensor(uint8_t pin, bool asButton = true);

	void begin();
	void tick();

	// Acesso por índice
	uint8_t count() const { return n_; }
	const SensorState& sensor(uint8_t i) const { return s_[i]; }
	bool button(uint8_t i) const { return (i < n_) ? s_[i].btnState : false; }

	bool consumePrintTick();
	String makeCsvHeader() const;
	String makeCsvRow() const;
	Snapshot get() const;

private:
	Config cfg_;

	SensorState s_[MAX_SENSORS];
	uint8_t n_ = 0;

	uint32_t nextSample_ = 0;
	uint32_t nextPrint_  = 0;
	bool printReady_ = false;

	static int clampi(int x, int a, int b);
	void sensorInit(SensorState& s);
	void sensorUpdate(SensorState& s);
	void updateVirtualButton(SensorState& s);

	void handleSerialCommands();
	void printAllCal() const;
};

} // namespace LightSensors
