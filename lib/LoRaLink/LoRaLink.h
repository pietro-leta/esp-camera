// =============================================================
// lib/lora_link/lora_link.h
// =============================================================
#pragma once

#include <Arduino.h>
#include <LoRa.h>

namespace LoRaLink {

struct RadioPins {
	int ss;
	int rst;
	int dio0;
	constexpr RadioPins(int ss_=27, int rst_=14, int dio0_=26)
		: ss(ss_), rst(rst_), dio0(dio0_) {}
};

struct RadioConfig {
	RadioPins pins;
	long freq_hz = 433E6;
};

bool beginRadio(const RadioConfig& cfg, bool retry_forever, Stream* log = nullptr);

// ---------------- Protocol ----------------
static constexpr uint8_t PKT_BEGIN = 0x01;
static constexpr uint8_t PKT_DATA  = 0x02;
static constexpr uint8_t PKT_END   = 0x03;
static constexpr uint8_t PKT_ACK   = 0x81;
static constexpr uint8_t PKT_NACK  = 0x82;
static constexpr uint8_t PKT_REQ   = 0x10;
static constexpr uint8_t PKT_GRANT = 0x11;
static constexpr uint8_t PKT_BUSY  = 0x12;

static constexpr uint16_t MAX_CHUNK = 200;
//static constexpr uint8_t  MAX_NAME_LEN = 31;

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF);
inline uint16_t rand16() { return (uint16_t)(esp_random() & 0xFFFF); }

// ---------------- Send helpers ----------------
void sendBegin(uint16_t tx_id, uint32_t img_id, uint16_t token,
							 uint32_t file_size, uint16_t chunk_size,
							 uint16_t total_chunks, const char* filename);

void sendEnd(uint16_t tx_id, uint32_t img_id, uint16_t token);

void sendData(uint16_t tx_id, uint32_t img_id, uint16_t token,
							uint16_t chunk_idx, const uint8_t* payload, uint16_t len);

bool readAck(uint16_t expected_tx_id, uint32_t expected_img_id,
						 uint16_t expected_chunk);

} // namespace LoRaLink
