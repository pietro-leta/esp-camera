#if defined(ROLE_RX)

#include <Arduino.h>
#include <LoRa.h>
#include <esp_system.h>
#include "LoRaLink.h"

// ===== LED simples: pisca a cada chunk encaminhado =====
// GPIO2 costuma ser o LED onboard no ESP32 DevKit
static const int LED_STATUS = 2;
static const uint16_t LED_PULSE_MS = 30;
static uint32_t g_led_pulse_until = 0;

static inline void led_pulse() {
	digitalWrite(LED_STATUS, HIGH);
	g_led_pulse_until = millis() + LED_PULSE_MS;
}

static inline void led_pulse_update() {
	if (g_led_pulse_until && (int32_t)(millis() - g_led_pulse_until) >= 0) {
		digitalWrite(LED_STATUS, LOW);
		g_led_pulse_until = 0;
	}
}


// ===== Ajuste pinos conforme seu hardware =====
static const int LORA_SS   = 27;  // NSS
static const int LORA_RST  = 14;
static const int LORA_DIO0 = 26;

static const long LORA_FREQ = 433E6;

// ===== Protocolo LoRa (TX->RX) =====
static const uint8_t PKT_BEGIN = 0x01;
static const uint8_t PKT_DATA  = 0x02;
static const uint8_t PKT_END   = 0x03;

static const uint8_t PKT_ACK  = 0x81;
static const uint8_t PKT_NACK = 0x82;

// ===== Handshake =====
static const uint8_t PKT_REQ   = 0x10; // TX->RX
static const uint8_t PKT_GRANT = 0x11; // RX->TX
static const uint8_t PKT_BUSY  = 0x12; // RX->TX

// ===== Tamanhos =====
static const uint16_t MAX_CHUNK = 200;

// ===== Protocolo Serial (RX->PC) =====
static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;

static const uint8_t SER_PKT_LOG   = 0x10;
static const uint8_t SER_PKT_HELLO = 0x11;

// ===== Serial (PC->RX) =====
// Heartbeat do PC para o RX liberar GRANTs
static const uint8_t SER_PKT_PC_HEARTBEAT = 0x20;

// ===== Parâmetros RX =====
static const uint16_t GRANT_VALID_MS = 3000;            // TX deve mandar BEGIN dentro disso
static const uint32_t SESSION_IDLE_TIMEOUT_MS = 60000;  // se travar, libera
static const uint16_t BUSY_RETRY_MS_DEFAULT = 5000;
static const uint16_t BUSY_RETRY_MS_JITTER  = 500;

// ===== Heartbeat gating =====
static const uint32_t PC_HEARTBEAT_TIMEOUT_MS = 4000; // se não chegar hb nesse prazo, bloqueia GRANT
static const uint16_t PC_NOT_READY_RETRY_MS   = 2000; // sugestão de retry p/ TX quando PC não pronto


// GENERAL HELPERS //
static uint8_t sanitize_fixed_name(const char* in, uint8_t in_len, char* out, uint8_t out_len) {
	for (uint8_t i = 0; i < out_len; i++) out[i] = '_';
	for (uint8_t i = 0; i < in_len && i < out_len; i++) {
		char c = in[i];
		if (c == '\0') continue;
		if (c == '/' || c == '\\') continue;
		if ((uint8_t)c < 32 || (uint8_t)c >= 127) continue;
		if (c == ':' || c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|') continue;
		out[i] = c;
	}
	out[out_len] = '\0';
	return out_len;
}


// ===== CRC =====
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF) {
	while (len--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (int i=0; i<8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
	}
	return crc;
}

static uint16_t rand16() { return (uint16_t)(esp_random() & 0xFFFF); }

// ===== Serial framing =====
static void serial_send_frame(const uint8_t* payload, uint16_t len) {
	uint8_t hdr[4] = {MAGIC0, MAGIC1, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
	Serial.write(hdr, 4);
	Serial.write(payload, len);
}

static void serial_send_log(const char* msg) {
	uint16_t mlen = (uint16_t)strlen(msg);
	if (mlen > 220) mlen = 220;

	static uint8_t payload[1 + 2 + 220];
	payload[0] = SER_PKT_LOG;
	payload[1] = (uint8_t)(mlen & 0xFF);
	payload[2] = (uint8_t)(mlen >> 8);
	memcpy(payload + 3, msg, mlen);

	serial_send_frame(payload, (uint16_t)(3 + mlen));
}

// =====================================================
// Serial RX (PC->RX): state machine para receber frames
// =====================================================
struct PcSerialRx {
	uint8_t state = 0;      // 0=wait M0, 1=wait M1, 2=lenL, 3=lenH, 4=payload
	uint16_t len = 0;
	uint16_t got = 0;
	uint8_t buf[64];        // suficiente pro heartbeat (bem pequeno)
};

static PcSerialRx g_pc_rx;
static uint32_t g_pc_last_hb_ms = 0;
static bool g_pc_ready = false;

static inline bool pc_is_ready() {
	if (!g_pc_ready) return false;
	return (millis() - g_pc_last_hb_ms) <= PC_HEARTBEAT_TIMEOUT_MS;
}

static void pc_set_ready(bool ready) {
	if (ready == g_pc_ready) return;
	g_pc_ready = ready;
	if (g_pc_ready) {
		serial_send_log("[RX] heartbeat recebido, pronto");
	} else {
		serial_send_log("[RX] heartbeat timeout, bloqueando GRANT");
	}
}

static void handle_pc_frame(const uint8_t* p, uint16_t n) {
	if (n < 1) return;
	uint8_t t = p[0];
	if (t == SER_PKT_PC_HEARTBEAT) {
		// payload opcional: uint32_t ms do PC (não necessário)
		g_pc_last_hb_ms = millis();
		pc_set_ready(true);
	}
}

static void poll_pc_serial() {
	while (Serial.available() > 0) {
		uint8_t b = (uint8_t)Serial.read();

		switch (g_pc_rx.state) {
			case 0: // wait MAGIC0
				if (b == MAGIC0) g_pc_rx.state = 1;
				break;
			case 1: // wait MAGIC1
				if (b == MAGIC1) g_pc_rx.state = 2;
				else g_pc_rx.state = 0;
				break;
			case 2: // len low
				g_pc_rx.len = b;
				g_pc_rx.state = 3;
				break;
			case 3: // len high
				g_pc_rx.len |= ((uint16_t)b << 8);
				g_pc_rx.got = 0;
				// por segurança, só aceitamos frames pequenos do PC
				if (g_pc_rx.len == 0 || g_pc_rx.len > sizeof(g_pc_rx.buf)) {
					g_pc_rx.state = 0;
					break;
				}
				g_pc_rx.state = 4;
				break;
			case 4: // payload
				g_pc_rx.buf[g_pc_rx.got++] = b;
				if (g_pc_rx.got >= g_pc_rx.len) {
					handle_pc_frame(g_pc_rx.buf, g_pc_rx.len);
					g_pc_rx.state = 0;
				}
				break;
			default:
				g_pc_rx.state = 0;
				break;
		}
	}

	// watchdog do heartbeat
	if (g_pc_ready && (millis() - g_pc_last_hb_ms > PC_HEARTBEAT_TIMEOUT_MS)) {
		pc_set_ready(false);
	}
}

// ===== LoRa replies =====
static void send_ack(uint8_t type, uint16_t tx_id, uint32_t img_id, uint16_t chunk_idx) {
	LoRa.beginPacket();
	LoRa.write(type);
	LoRa.write((uint8_t*)&tx_id, sizeof(tx_id));
	LoRa.write((uint8_t*)&img_id, sizeof(img_id));
	LoRa.write((uint8_t*)&chunk_idx, sizeof(chunk_idx));
	LoRa.endPacket(true);
}

static void send_grant(uint16_t tx_id, uint16_t token, uint16_t valid_ms) {
	// GRANT: type(1) tx_id(2) token(2) valid_ms(2)
	uint8_t tmp[1 + 2 + 2 + 2];
	size_t off=0;
	tmp[off++] = PKT_GRANT;
	memcpy(tmp+off, &tx_id, 2); off+=2;
	memcpy(tmp+off, &token, 2); off+=2;
	memcpy(tmp+off, &valid_ms, 2); off+=2;

	LoRa.beginPacket();
	LoRa.write(tmp, sizeof(tmp));
	LoRa.endPacket(true);
}

static void send_busy(uint16_t tx_id, uint16_t retry_ms) {
	// BUSY: type(1) tx_id(2) retry_ms(2)
	uint8_t tmp[1 + 2 + 2];
	size_t off=0;
	tmp[off++] = PKT_BUSY;
	memcpy(tmp+off, &tx_id, 2); off+=2;
	memcpy(tmp+off, &retry_ms, 2); off+=2;

	LoRa.beginPacket();
	LoRa.write(tmp, sizeof(tmp));
	LoRa.endPacket(true);
}

// ===== Sessão (1 arquivo por vez) =====
struct RxSession {
	bool active = false;
	bool granted = false;

	uint16_t tx_id = 0;
	uint32_t img_id = 0;
	uint16_t token = 0;

	uint32_t grant_deadline_ms = 0;
	uint32_t last_activity_ms = 0;

	// === tracking do arquivo atual (para auto-release) ===
	bool begun = false;
	uint16_t total_chunks = 0;
	uint16_t next_chunk = 0; // esperamos receber este idx (sequencial)
};

static RxSession g_sess;

static void reset_session(const char* reason) {
	g_sess = RxSession();
	if (reason) serial_send_log(reason);
}

// ===== AUTH separado: trata PKT_REQ =====
static void handle_req() {
	uint16_t tx_id;
	LoRa.readBytes((uint8_t*)&tx_id, 2);

	// ===== NOVO: só libera GRANT se o PC estiver com heartbeat recente =====
	if (!pc_is_ready()) {
		// Não abre sessão; manda BUSY com retry curto
		uint16_t retry_ms = PC_NOT_READY_RETRY_MS + (rand16() % 300);
		send_busy(tx_id, retry_ms);

		char msg[120];
		snprintf(msg, sizeof(msg), "[RX] PC not ready -> BUSY tx=%u retry=%u ms",
						 (unsigned)tx_id, (unsigned)retry_ms);
		serial_send_log(msg);
		return;
	}

	if (!g_sess.active) {
		g_sess.active = true;
		g_sess.granted = true;
		g_sess.tx_id = tx_id;
		g_sess.img_id = 0;
		g_sess.token = rand16();
		g_sess.grant_deadline_ms = millis() + GRANT_VALID_MS;
		g_sess.last_activity_ms = millis();

		send_grant(tx_id, g_sess.token, GRANT_VALID_MS);

		char msg[140];
		snprintf(msg, sizeof(msg), "[RX] GRANT tx=%u token=%u",
						 (unsigned)tx_id, (unsigned)g_sess.token);
		serial_send_log(msg);
	} else {
		uint16_t retry_ms = BUSY_RETRY_MS_DEFAULT + (rand16() % BUSY_RETRY_MS_JITTER);
		send_busy(tx_id, retry_ms);

		char msg[140];
		snprintf(msg, sizeof(msg), "[RX] BUSY tx=%u retry=%u ms",
						 (unsigned)tx_id, (unsigned)retry_ms);
		serial_send_log(msg);
	}
}

static uint32_t lastHello = 0;

void setup() {
	pinMode(LED_STATUS, OUTPUT);
	digitalWrite(LED_STATUS, LOW);

	Serial.begin(115200);
	delay(300);
	Serial.print("Iniciando RX");

	delay(300);
	LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
	delay(300);

	if (!LoRa.begin(LORA_FREQ)) {
		while (true) {
			serial_send_log("[RX] LoRa.begin FAIL (retrying...)");
			delay(1000);
			LoRa.end();
			delay(200);

			delay(300);
			LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
			delay(300);

			if (LoRa.begin(LORA_FREQ)) break;
		}
	}

	serial_send_log("[RX] boot ok (N files, per-file grant)");
	serial_send_log("[RX] aguardando heartbeat do PC para liberar GRANT...");
}

void loop() {
	// LED pulse housekeeping
	led_pulse_update();

	// PC heartbeat + watchdog (PC->RX)
	poll_pc_serial();

	// Se o PC cair no meio de uma sessão, aborta para evitar "sucesso" sem salvar.
	if (g_sess.active && !pc_is_ready()) {
		reset_session("[RX] PC heartbeat lost during session -> released");
	}
	if (millis() - lastHello > 2000) {
		lastHello = millis();
		uint8_t p[1+4];
		p[0] = SER_PKT_HELLO;
		uint32_t t = millis();
		memcpy(p+1, &t, 4);
		serial_send_frame(p, sizeof(p));
	}

	if (g_sess.active && (millis() - g_sess.last_activity_ms > SESSION_IDLE_TIMEOUT_MS)) {
		reset_session("[RX] session timeout -> released");
	}

	int packetSize = LoRa.parsePacket();
	if (!packetSize) return;

	uint8_t type = LoRa.read();

	if (type == PKT_REQ) {
		handle_req();
		return;
	}


	if (type == PKT_BEGIN) {
		uint16_t tx_id; uint32_t img_id; uint16_t token;
		uint32_t file_size; uint16_t chunk_size; uint16_t total_chunks;

		uint8_t name_len = 0;
		const uint8_t fixed_len = 10;
		char fname[fixed_len + 1] = {0};

		LoRa.readBytes((uint8_t*)&tx_id, 2);
		LoRa.readBytes((uint8_t*)&img_id, 4);
		LoRa.readBytes((uint8_t*)&token, 2);
		LoRa.readBytes((uint8_t*)&file_size, 4);
		LoRa.readBytes((uint8_t*)&chunk_size, 2);
		LoRa.readBytes((uint8_t*)&total_chunks, 2);

		// lê nome
		name_len = (uint8_t)LoRa.read();
		(void)name_len;
		LoRa.readBytes((uint8_t*)fname, fixed_len);
		fname[fixed_len] = 0;

		if (!g_sess.active || !g_sess.granted ||
				tx_id != g_sess.tx_id || token != g_sess.token) {
			serial_send_log("[RX] BEGIN rejected (no valid grant/token)");
			return;
		}

		if ((int32_t)(millis() - g_sess.grant_deadline_ms) > 0) {
			reset_session("[RX] BEGIN too late -> released");
			return;
		}

		if (g_sess.img_id == 0) {
			g_sess.img_id = img_id;
		} else if (img_id != g_sess.img_id) {
			serial_send_log("[RX] BEGIN rejected (img_id mismatch)");
			return;
		}

		g_sess.last_activity_ms = millis();

		// tracking
		g_sess.begun = true;
		g_sess.total_chunks = total_chunks;
		g_sess.next_chunk = 0;

		// encaminha BEGIN pro PC (agora inclui tx_id + filename)
		// payload: 0x01 tx_id(2) img_id(4) file_size(4) chunk_size(2) total_chunks(2) name_len(1) name(name_len)
		char clean_name[fixed_len + 1] = {0};
		uint8_t clean_len = sanitize_fixed_name(fname, fixed_len, clean_name, fixed_len);

		uint8_t payload[1 + 2 + 4 + 4 + 2 + 2 + 1 + 10];
		size_t off = 0;
		payload[off++] = 0x01;
		memcpy(payload+off, &tx_id, 2); off+=2;
		memcpy(payload+off, &img_id, 4); off+=4;
		memcpy(payload+off, &file_size, 4); off+=4;
		memcpy(payload+off, &chunk_size, 2); off+=2;
		memcpy(payload+off, &total_chunks, 2); off+=2;
		payload[off++] = clean_len;
		memcpy(payload+off, clean_name, clean_len); off += clean_len;

		serial_send_frame(payload, (uint16_t)off);

		char msg[120];
		snprintf(msg, sizeof(msg), "[RX] BEGIN ok fname=%s", (clean_len?clean_name:"(none)"));
		serial_send_log(msg);
		return;
	}


	if (type == PKT_DATA) {
		uint16_t tx_id; uint32_t img_id; uint16_t token;
		uint16_t chunk_idx; uint16_t payload_len;

		LoRa.readBytes((uint8_t*)&tx_id, 2);
		LoRa.readBytes((uint8_t*)&img_id, 4);
		LoRa.readBytes((uint8_t*)&token, 2);
		LoRa.readBytes((uint8_t*)&chunk_idx, 2);
		LoRa.readBytes((uint8_t*)&payload_len, 2);

		if (payload_len > MAX_CHUNK) return;

		static uint8_t buf[MAX_CHUNK];
		LoRa.readBytes(buf, payload_len);

		uint16_t rx_crc;
		LoRa.readBytes((uint8_t*)&rx_crc, 2);

		if (!g_sess.active || !g_sess.granted ||
				tx_id != g_sess.tx_id || img_id != g_sess.img_id || token != g_sess.token) {
			return;
		}

		uint16_t calc = crc16_ccitt(buf, payload_len);
		if (calc != rx_crc) {
			send_ack(PKT_NACK, tx_id, img_id, chunk_idx);
			return;
		}


		send_ack(PKT_ACK, tx_id, img_id, chunk_idx);
		g_sess.last_activity_ms = millis();

		// encaminha DATA pro PC
		static uint8_t out[1 + 4 + 2 + 2 + MAX_CHUNK + 2];
		size_t off = 0;
		out[off++] = 0x02;
		memcpy(out+off, &img_id, 4); off+=4;
		memcpy(out+off, &chunk_idx, 2); off+=2;
		memcpy(out+off, &payload_len, 2); off+=2;
		memcpy(out+off, buf, payload_len); off+=payload_len;
		memcpy(out+off, &rx_crc, 2); off+=2;

		serial_send_frame(out, (uint16_t)off);

		// pisca LED a cada chunk encaminhado
		led_pulse();

		// === novo: tracking + auto-release ===
		// Como o TX só manda o próximo chunk após ACK, esperamos sequência perfeita.
		// Se vier duplicado (retry), não avançamos.
		if (g_sess.begun && chunk_idx == g_sess.next_chunk) {
			g_sess.next_chunk++;

			if (g_sess.next_chunk >= g_sess.total_chunks) {
				// manda END pro PC (0x03) e libera sessão
				uint8_t pend[1+4];
				pend[0] = 0x03;
				memcpy(pend+1, &img_id, 4);
				serial_send_frame(pend, sizeof(pend));

				serial_send_log("[RX] auto-release (last chunk) -> session released");
				reset_session(nullptr);
			}
		}

		return;

	}

	if (type == PKT_END) {
		uint16_t tx_id; uint32_t img_id; uint16_t token; uint16_t hdr_crc;
		LoRa.readBytes((uint8_t*)&tx_id, 2);
		LoRa.readBytes((uint8_t*)&img_id, 4);
		LoRa.readBytes((uint8_t*)&token, 2);
		LoRa.readBytes((uint8_t*)&hdr_crc, 2);

		uint8_t tmp[1+2+4+2];
		tmp[0] = PKT_END;
		memcpy(tmp+1, &tx_id, 2);
		memcpy(tmp+3, &img_id, 4);
		memcpy(tmp+7, &token, 2);

		if (crc16_ccitt(tmp, sizeof(tmp)) != hdr_crc) {
			serial_send_log("[RX] END CRC mismatch (ignored)");
			return;
		}

		if (!g_sess.active || !g_sess.granted ||
				tx_id != g_sess.tx_id || img_id != g_sess.img_id || token != g_sess.token) {
			return;
		}

		g_sess.last_activity_ms = millis();

		uint8_t payload[1+4];
		payload[0] = 0x03;
		memcpy(payload+1, &img_id, 4);
		serial_send_frame(payload, sizeof(payload));

		serial_send_log("[RX] END forwarded to PC -> session released");
		reset_session(nullptr);
		return;
	}
}

#endif