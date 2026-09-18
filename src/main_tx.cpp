// ========================= src/main_tx.cpp =========================
// Compile com -DROLE_TX no build_flags do PlatformIO.
#ifdef ROLE_TX
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "DeepSleep.h"
#include "SdLogger.h"
#include "LightSensors.h"
#include "Maintenance.h"
#include "ThumbMaker.h"
#include "LoRaLink.h"

static SdLogger::Logger* g_log = nullptr;
static LightSensors::Manager* g_sensors = nullptr;
static Maintenance::Controller* g_maint = nullptr;
static ThumbMaker::Maker* g_maker = nullptr;

// Controla disparo por borda (0->1) do botão virtual do sensor no PIN 32
static bool g_lastLight32 = false;
static int findSensorIndexByPin(LightSensors::Manager* sensors, int pin) {
	if (!sensors) return -1;
	const uint8_t n = sensors->count();
	for (uint8_t i = 0; i < n; i++) {
	  if ((int)sensors->sensor(i).pin == pin) return (int)i;
	}
  	return -1;
}

// Grava se houve execução do thumbmaker (thumb esperando envio)
static bool g_thumbWait = false;
using namespace LoRaLink;

// ------------------------- Identidade do TX -------------------------
static constexpr uint16_t TX_ID = 1;

// ------------------------- SD -------------------------
static constexpr int SD_CS = 5;
static const char* SRC_DIR  = "/THUMB";
static const char* SENT_DIR = "/THUMB_sent";

// ------------------------- Parâmetros TX -------------------------
static constexpr uint16_t CHUNK_SIZE = 200;
static constexpr uint32_t REQ_TIMEOUT_MS = 1200;
// ACK = acknowledgement, o rx conirma quando terminou de receber o chunk
static constexpr uint32_t ACK_TIMEOUT_MS = 700; 
static constexpr uint8_t  MAX_RETRIES_CHUNK = 6;
static constexpr uint8_t  MAX_RETRIES_REQ   = 10;
static constexpr uint16_t REQ_JITTER_MS  = 500;
static constexpr uint16_t BUSY_JITTER_MS = 300;
constexpr unsigned long SCAN_INTERVAL_MS = 10000;
unsigned long lastScanMs = 0;

// ------------------------- Handshake TX (parsing pequeno) -------------------------
struct HResp {
	bool got = false;
	bool grant = false;
  	uint16_t token = 0;
	uint16_t valid_ms = 0;
  	uint16_t retry_ms = 0;
};
static HResp wait_handshake() {
	HResp r;
	uint32_t start = millis();
	while (millis() - start < REQ_TIMEOUT_MS) {
		int n = LoRa.parsePacket();
		if (!n) { delay(1); continue; }
		
		uint8_t type = LoRa.read();
		
		if (type == PKT_GRANT) {
			uint16_t tx_id; uint16_t token; uint16_t valid_ms;
			LoRa.readBytes((uint8_t*)&tx_id, 2);
			LoRa.readBytes((uint8_t*)&token, 2);
			LoRa.readBytes((uint8_t*)&valid_ms, 2);
			if (tx_id != TX_ID) continue;
			r.got = true; r.grant = true; r.token = token; r.valid_ms = valid_ms;
			return r;
		}
	
		if (type == PKT_BUSY) {
			uint16_t tx_id; uint16_t retry_ms;
			LoRa.readBytes((uint8_t*)&tx_id, 2);
			LoRa.readBytes((uint8_t*)&retry_ms, 2);
			if (tx_id != TX_ID) continue;
			r.got = true; r.grant = false; r.retry_ms = retry_ms;
			return r;
		}
	}
	return r;
}
static void send_request() {
	uint8_t tmp[1 + 2];
	tmp[0] = PKT_REQ;
	memcpy(tmp + 1, &TX_ID, 2);
	LoRa.beginPacket();
	LoRa.write(tmp, sizeof(tmp));
	LoRa.endPacket(true);
}

static bool acquire_grant(uint16_t& out_token, uint16_t& out_valid_ms) {
	delay(rand16() % REQ_JITTER_MS);
	for (uint8_t attempt = 0; attempt < MAX_RETRIES_REQ; attempt++) {
		send_request();
		HResp r = wait_handshake();
		if (!r.got) {
			uint16_t wait_ms = 2000 + (rand16() % 2000);
			Serial.printf("[TX] no response, retry in %u ms\n", wait_ms);
			delay(wait_ms);
			continue;
		}
		
		if (r.grant) {
			out_token = r.token;
			out_valid_ms = r.valid_ms;
			Serial.printf("[TX] GRANT token=%u valid=%u ms\n", out_token, out_valid_ms);
			return true;
		} else {
			uint16_t wait_ms = r.retry_ms + (rand16() % BUSY_JITTER_MS);
			Serial.printf("[TX] BUSY, retry in %u ms\n", wait_ms);
			delay(wait_ms);
		}
	}
	return false;
}

// ------------------------- ACK wait (TX) -------------------------
static bool wait_ack(uint32_t img_id, uint16_t chunk_idx) {
	// ack = acknowledgement
	uint32_t start = millis();
	while (millis() - start < ACK_TIMEOUT_MS) {
		int n = LoRa.parsePacket();
		if (!n) { delay(1); continue; }
		
		uint8_t type = LoRa.read();
		if (type != PKT_ACK && type != PKT_NACK) continue;
		
		// Check if it's the right ACK for our specific chunk
		if (!LoRaLink::readAck(TX_ID, img_id, chunk_idx)) continue;
		
		return (type == LoRaLink::PKT_ACK); 
		// Removed the double return statement here
	}
	return false;
}

// ------------------------- File transfer -------------------------
static bool transfer_file_with_token(
	File& f,
	const char* fname,
	uint32_t img_id,
	uint16_t token,
	uint32_t file_size,
	uint16_t total_chunks ){

	sendBegin(
		TX_ID,
		img_id,
		token,
		file_size,
		CHUNK_SIZE,
		total_chunks,
		fname            // ex: "IMG_000123.bin"
	);

	delay(50);
	static uint8_t buf[CHUNK_SIZE];
	
	for (uint16_t idx = 0; idx < total_chunks; idx++) {
		uint32_t offset = (uint32_t)idx * (uint32_t)CHUNK_SIZE;
		f.seek(offset);
		int n = f.read(buf, CHUNK_SIZE);
		if (n <= 0) {
			Serial.printf("[TX] read fail chunk=%u\n", idx);
			return false;
		}
		
		bool ok = false;
		for (uint8_t attempt = 0; attempt < MAX_RETRIES_CHUNK; attempt++) {
			sendData(TX_ID, img_id, token, idx, buf, (uint16_t)n);
			if (wait_ack(img_id, idx)) { ok = true; break; }
		}
		
		if (!ok) {
			Serial.printf("[TX] FAIL chunk=%u\n", idx);
			return false;
		}
	}
	
	sendEnd(TX_ID, img_id, token);
	return true;
}
// ------------------------- Utils SD (iguais ao seu) -------------------------
static bool ensureDir(const char* path) {
	if (SD.exists(path)) return true;
	String p(path);
	if (!p.startsWith("/")) p = "/" + p;
	String cur = "";
	
	for (size_t i = 0; i < p.length(); i++) {
		char c = p[i];
		cur += c;
		if (c == '/' && cur.length() > 1) {
			if (!SD.exists(cur.c_str())) SD.mkdir(cur.c_str());
		}

	}
	if (!SD.exists(p.c_str())) return SD.mkdir(p.c_str());
	return true;
}

static bool moveToSent(const char* src_full) {
	String src(src_full);
	String prefix(SRC_DIR);
	if (!src.startsWith(prefix)) return false;
	String rel = src.substring(prefix.length());
	String dst = String(SENT_DIR) + rel;
	int lastSlash = dst.lastIndexOf('/');
	if (lastSlash > 0) {
		String ddir = dst.substring(0, lastSlash);
		ensureDir(ddir.c_str());
	} else {
		ensureDir(SENT_DIR);
	}
	
	if (SD.exists(dst.c_str())) {
		int dot = dst.lastIndexOf('.');
		String base = (dot > 0) ? dst.substring(0, dot) : dst;
		String ext  = (dot > 0) ? dst.substring(dot) : "";
		for (int i = 1; i < 1000; i++) {
			String cand = base + "_" + String(i) + ext;
			if (!SD.exists(cand.c_str())) { dst = cand; break; }
		}
	}
	
	bool ok = SD.rename(src.c_str(), dst.c_str());
	Serial.printf(ok ? "[TX] moved -> %s\n" : "[TX] move FAIL (%s -> %s)\n",
					dst.c_str(), src.c_str(), dst.c_str());
	return ok;
}

static String pathJoin(const String& a, const String& b) {
	if (a.length() == 0) return b;
	if (b.length() == 0) return a;

	String A = a, B = b;
	if (!A.startsWith("/")) A = "/" + A;
	if (A.endsWith("/")) A.remove(A.length() - 1);
	if (B.startsWith("/")) B = B.substring(1);
	return A + "/" + B;
}

static bool send_one_file(const char* full_path) {
	File f = SD.open(full_path, FILE_READ);
	if (!f) {
		Serial.printf("[TX] cannot open %s\n", full_path);
		return false;
	}

	uint32_t file_size = f.size();
	if (file_size == 0) { f.close(); return false; }
	uint16_t total_chunks = (file_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
	uint32_t img_id = (uint32_t)millis();

	Serial.printf("\n[TX] SEND %s size=%lu chunks=%u\n",
					full_path, (unsigned long)file_size, (unsigned)total_chunks);


	uint16_t token=0, valid_ms=0;
	if (!acquire_grant(token, valid_ms)) {
		Serial.println("[TX] failed to acquire grant");
		f.close();
		return false;
	}

	const char* fname = strrchr(full_path, '/');
	fname = fname ? fname + 1 : full_path;
	bool ok = transfer_file_with_token(
		f,
		fname,        // agora no escopo certo
		img_id,
		token,
		file_size,
		total_chunks
	);
	f.close();
	Serial.println(ok ? "[TX] DONE file" : "[TX] FAIL file");
	return ok;
}

static void scan_and_send_dir(const char* dirPath) {
	File dir = SD.open(dirPath);
	if (!dir || !dir.isDirectory()) {
		Serial.printf("[TX] not a dir: %s\n", dirPath);
		return;
	}
	while (true) {
		File entry = dir.openNextFile();
		if (!entry) break;
		String name = entry.name();
		bool isDir = entry.isDirectory();
		uint32_t sz = entry.size();
		entry.close();
		String fullPath = pathJoin(String(dirPath), name);
		if (isDir) {
			scan_and_send_dir(fullPath.c_str());
			continue;
		}
		
		if (sz == 0) {
			Serial.printf("[TX] skip empty: %s\n", fullPath.c_str());
			continue;
		}
		
		bool ok = send_one_file(fullPath.c_str());
		if (ok) {
			moveToSent(fullPath.c_str());
		} else {
			Serial.printf("[TX] keep (send failed): %s\n", fullPath.c_str());
			delay(1000);
		}
	}
	dir.close();
}

void setup() { 
	Serial.begin(115200);
	delay(300);
	Serial.println("INICIANDO TX");
	
	// -- Deep Sleep --
	{
		DeepSleep::setupDeepSleep();
	}
	Serial.println("Deep Sleep inicializado");
	
	// --- SD logger ---
	{
		SdLogger::Config logCfg;
		logCfg.sdCs = 5;
		logCfg.spiFreq = 16000000;
		logCfg.verbose = true;
		static SdLogger::Logger logger(logCfg);
		g_log = &logger;
		g_log->begin(); // se falhar, segue sem SD
	}
	Serial.println("Sd Logger inicializado");
	
	// --- sensores ---
	{
		LightSensors::Config sensCfg;
		sensCfg.verbose = true;
		static LightSensors::Manager sensors(sensCfg);
		g_sensors = &sensors;
		// Sensores com "virtual button"
		// addSensor(pin, isButton)
		g_sensors->addSensor(32, true); // <- gatilho do ThumbMaker
		g_sensors->addSensor(34, true);
		g_sensors->addSensor(35, true); // <- gatilho default do Maintenance (se você quiser)
		g_sensors->begin();
		
		// (opcional) header do CSV uma vez
		if (g_log && g_log->ready()) {
			g_log->logSensorsRow(g_sensors->makeCsvHeader());
		}
	}
	Serial.println("Sensores inicializados");
	
	// --- ThumbMaker ---
	{
		ThumbMaker::Config tCfg;
		tCfg.verbose = true;
		static ThumbMaker::Maker maker(tCfg);
		g_maker = &maker;
		if (!g_maker->begin()) {
			Serial.println("[ThumbMaker] begin() falhou; seguindo sem conversao de thumbnails.");
		}
		
		// --- manutenção (wifi/ota/http/leds) ---
		Maintenance::Config mCfg;
		mCfg.ssid = "CRISTIAN";
		mCfg.pass = "csa051721";
		mCfg.otaHostname = "esp32-node01";
		mCfg.otaPassword = "senha_ota_forte";
		
		// Gatilho do maintenance (como combinamos)
		mCfg.maintBtn = GPIO_NUM_13;
		mCfg.trigger = Maintenance::Config::TriggerSource::Either;
		mCfg.triggerLightPin = 35;
		static Maintenance::Controller maint(mCfg, g_log, g_sensors);
		g_maint = &maint;
		g_maint->begin();
		
		// Inicializa estado do "botão virtual" do PIN 32
		const int idx32 = findSensorIndexByPin(g_sensors, 32);
		g_lastLight32 = (idx32 >= 0) ? g_sensors->button((uint8_t)idx32) : false;
	}
	Serial.println("Thumb Maker inicializado");
	
	// LoRa
	{ 
		RadioConfig rc;
		rc.pins = RadioPins(27, 14, 26);
		rc.freq_hz = 433E6;
		delay(1000);
		if (!beginRadio(rc, /*retry_forever=*/false, &Serial)) {
			Serial.println("[TX] LoRa.begin FAIL");
			while(true) delay(1000);
		}
		
		Serial.println("[TX] Ready (N files, per-file grant)");
		ensureDir(SENT_DIR);
		
		//  Serial.println("[TX] Executando scan inicial");
		//  scan_and_send_dir(SRC_DIR);
		Serial.println("\n---> Terminando setup. Iniciando loop <---\n");
	}
	Serial.println("Lora inicializado");
}

void loop() {
	// Timer - Registro do tempo atual
	unsigned long now = millis();
	
	// 1) sensores sempre
	g_sensors->tick();
	
	// 2) Dispara ThumbMaker quando "botão de luz" do PIN 32 ligar (borda 0->1)
	if (g_maker) {
		Serial.println("g_maker inicializado.");
		const int idx32 = findSensorIndexByPin(g_sensors, 32);
		if (idx32 >= 0) {
			const bool light32 = g_sensors->button((uint8_t)idx32);
			
			Serial.print("sensor: "); Serial.print(idx32);
			Serial.print(" de ");Serial.println(g_sensors->count());
			if (light32) Serial.println("light: true");
			else Serial.println("light: false");

			if (g_lastLight32) Serial.print("last_light: true");
			else Serial.println("last_light: false");

			// borda de subida: OFF -> ON
			//if (!g_lastLight32 && light32) {
			// nao esta caindo aqui, ver isso dps
			if (true) {
				Serial.println("[ThumbMaker] Trigger pelo sensor (PIN 32). Processando waitlist...");
				g_maker->processWaitlist();
				Serial.println("[ThumbMaker] Fim do processamento.");
				// Grava que tem arquivo na fila para envio
				g_thumbWait = true;    
			}
			g_lastLight32 = light32;
		}
	} else {
		Serial.println("g_maker não inicializado.");
	}

	// 3) log 1Hz quando pronto
	if (g_log && g_log->ready() && g_sensors->consumePrintTick()) {
		g_log->logSensorsRow(g_sensors->makeCsvRow());
	}
	
	// 4) wifi/ota/http/leds sempre
	g_maint->tick();
	
	// SD Scan e Envio por LoRa
	// Scan programado - Executa se o thumbmaker tiver processado algo
	if (g_thumbWait){
		scan_and_send_dir(SRC_DIR);
		g_thumbWait = false;    
		Serial.println("\n[TX] Finished scan.");
	}
	
	// Scan regular - Executa scan em intervalos independentemente de qualquer coisa (flush)
	if (now - lastScanMs >= SCAN_INTERVAL_MS) {
		lastScanMs = now;
		scan_and_send_dir(SRC_DIR);
		Serial.println("\n[TX] Finished scan.");
	}

	// depois que fez tudo vai pro deep sleep
	Serial.println("Indo para Deep Sleep");
	DeepSleep::goDeepSleep();
}
#endif // ROLE_TX
