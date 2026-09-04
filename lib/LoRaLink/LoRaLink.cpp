// =============================================================
// lib/lora_link/lora_link.cpp
// =============================================================
#include "LoRaLink.h"

namespace LoRaLink {

// não sei o que isso faz
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc) {
	while (len--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (int i=0;i<8;i++)
			crc = (crc & 0x8000) ? (crc<<1)^0x1021 : (crc<<1);
	}
	return crc;
}

// inicia a comunicação
bool beginRadio(const RadioConfig& cfg, bool retry_forever, Stream* log) {
	LoRa.setPins(cfg.pins.ss, cfg.pins.rst, cfg.pins.dio0);
	delay(200);
	if (LoRa.begin(cfg.freq_hz)) return true;
	if (!retry_forever) return false;
	while (true) {
		if (log) log->println("[LoRa] begin FAIL, retrying");
		delay(1000);
		LoRa.end();
		delay(200);
		LoRa.setPins(cfg.pins.ss, cfg.pins.rst, cfg.pins.dio0);
		delay(200);
		if (LoRa.begin(cfg.freq_hz)) return true;
	}
}

void sendBegin(uint16_t tx_id, uint32_t img_id, uint16_t token,
							uint32_t file_size, uint16_t chunk_size,
							uint16_t total_chunks, const char* filename) {

	const uint8_t name_len = 10;
	char name_fixed[10];
	for (uint8_t i = 0; i < name_len; i++) name_fixed[i] = '_';
	if (filename) {
		// min retorna o menor parametro
		uint8_t n = min((uint8_t)strlen(filename), name_len);
		// se o nome for vazio ele usa um ______ ao invés do nome
		// memcopy é a função de C pra copiar uma string pra outra
		if (n) memcpy(name_fixed, filename, n);
	}

	// copia tudo nesse buf e manda pro lora
	uint8_t buf[1+2+4+2+4+2+2+1+name_len];
	// offset
	size_t o = 0;
	
	buf[o++] = PKT_BEGIN;
	memcpy(buf+o, &tx_id,2); o+=2;
	memcpy(buf+o, &img_id,4); o+=4;
	memcpy(buf+o, &token,2); o+=2;
	memcpy(buf+o, &file_size,4); o+=4;
	memcpy(buf+o, &chunk_size,2); o+=2;
	memcpy(buf+o, &total_chunks,2); o+=2;
	buf[o++]=name_len;
	memcpy(buf+o, name_fixed, name_len); o+=name_len;

	LoRa.beginPacket();
	LoRa.write(buf,o);
	LoRa.endPacket(true);
}

void sendEnd(uint16_t tx_id, uint32_t img_id, uint16_t token) {
	uint8_t buf[1+2+4+2];
	size_t o=0;
	
	buf[o++]=PKT_END;
	memcpy(buf+o,&tx_id,2); o+=2;
	memcpy(buf+o,&img_id,4); o+=4;
	memcpy(buf+o,&token,2); o+=2;
	uint16_t crc=crc16_ccitt(buf,o);

	LoRa.beginPacket();
	LoRa.write(buf,o);
	LoRa.write((uint8_t*)&crc,2);
	LoRa.endPacket(true);
}

void sendData(uint16_t tx_id, uint32_t img_id, uint16_t token,
							uint16_t chunk_idx, const uint8_t* payload, uint16_t len) {
	uint16_t crc = crc16_ccitt(payload,len);
	LoRa.beginPacket();
	LoRa.write(PKT_DATA);
	LoRa.write((uint8_t*)&tx_id,2);
	LoRa.write((uint8_t*)&img_id,4);
	LoRa.write((uint8_t*)&token,2);
	LoRa.write((uint8_t*)&chunk_idx,2);
	LoRa.write((uint8_t*)&len,2);
	LoRa.write(payload,len);
	LoRa.write((uint8_t*)&crc,2);
	LoRa.endPacket(true);
}

// ack = acknowledgement, o RX confirma quando recebeu
bool readAck(uint16_t expected_tx_id, uint32_t expected_img_id,
						uint16_t expected_chunk) {
	uint16_t tx;
	uint32_t img;
	uint16_t ch;

	LoRa.readBytes((uint8_t*)&tx,2);
	LoRa.readBytes((uint8_t*)&img,4);
	LoRa.readBytes((uint8_t*)&ch,2);
	
	return (tx == expected_tx_id && img == expected_img_id && ch == expected_chunk);
}

} // namespace LoRaLink