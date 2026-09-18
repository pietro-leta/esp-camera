#include "ThumbMaker.h"

#include <SPI.h>
#include <SD.h>
#include <TJpg_Decoder.h>

namespace ThumbMaker {

// Ponteiro pro objeto em processamento (TJpgDec callback é C-style)
Maker* Maker::active_ = nullptr;

Maker::Maker(const Config& cfg) : cfg_(cfg) {}

bool Maker::begin() {
	SPI.begin();

	// SD.begin(cs, spi, freq) (assinatura do ESP32/Arduino geralmente suporta freq)
	// Se sua core não suportar essa assinatura, troque para SD.begin(cfg_.sdCs).
	if (!SD.begin(cfg_.sdCs, SPI, cfg_.spiFreq)) {
		Serial.println("Erro: Cartao SD nao detectado!");
		return false;
	}

	if (!SD.exists(cfg_.outDir)) {
		if (!SD.mkdir(cfg_.outDir)) {
			Serial.println("Erro: nao consegui criar pasta de THUMB!");
			return false;
		}
	}

	return true;
}

bool Maker::endsWithJpg(const String& name) {
	String s = name;
	s.toLowerCase();
	return s.endsWith(".jpg") || s.endsWith(".jpeg");
}

inline uint8_t Maker::rgb565ToGray4(uint16_t c) {
	// mesma ideia que você usou, só mantendo 4-bit [0..15]
	uint32_t r = (c >> 11) & 0x1F;
	uint32_t g = (c >> 5)  & 0x3F;
	uint32_t b = (c)       & 0x1F;

	uint32_t r8 = (r * 255) / 31;
	uint32_t g8 = (g * 255) / 63;
	uint32_t b8 = (b * 255) / 31;

	uint32_t gray8 = (r8 * 30 + g8 * 59 + b8 * 11) / 100; // [0..255]
	return (uint8_t)(gray8 >> 4); // [0..15]
}

inline bool Maker::bitGetAt(const uint8_t* bits, uint16_t i) {
	return (bits[i >> 3] >> (i & 7)) & 1;
}

inline void Maker::bitSetAt(uint8_t* bits, uint16_t i) {
	bits[i >> 3] |= (1U << (i & 7));
}

bool Maker::ensureRowCacheAllocated(uint16_t newDecW) {
	// realloc das estruturas conforme a largura muda
	if (newDecW == 0) return false;

	// (re)aloca array de rows se necessário
	if (!rows_) {
		// aloca espaço pra 48 de RowCache
		rows_ = (RowCache*)calloc(cfg_.cacheRows, sizeof(RowCache));
		// se deu pau cai fora
		if (!rows_) return false;
	}

	filledBytes_ = (newDecW + 7) / 8;

	for (uint16_t i = 0; i < cfg_.cacheRows; i++) {
		rows_[i].data = (uint8_t*)realloc(rows_[i].data, newDecW);
		rows_[i].filled = (uint8_t*)realloc(rows_[i].filled, filledBytes_);
		if (!rows_[i].data || !rows_[i].filled) return false;

		memset(rows_[i].filled, 0, filledBytes_);
		rows_[i].filledCount = 0;
		rows_[i].y = -1;
	}

	return true;
}

void Maker::resetRow(uint16_t y) {
	RowCache* r = &rows_[y % cfg_.cacheRows];
	r->y = -1;
	r->filledCount = 0;
	if (r->filled && filledBytes_ > 0) memset(r->filled, 0, filledBytes_);
}

int Maker::freeCaches() {
	if (!rows_) return 1;
	for (uint16_t i = 0; i < cfg_.cacheRows; i++) {
		free(rows_[i].data);
		free(rows_[i].filled);
		rows_[i].data = nullptr;
		rows_[i].filled = nullptr;
	}
	free(rows_);
	rows_ = nullptr;
	return 0;
}

uint8_t Maker::getAverage2x2_4bit(RowCache* r0, RowCache* r1, uint16_t x) {
	uint32_t sum = r0->data[x];
	sum += (x + 1 < decW_) ? r0->data[x + 1] : r0->data[x];

	if (r1) {
		sum += r1->data[x];
		sum += (x + 1 < decW_) ? r1->data[x + 1] : r1->data[x];
		return (uint8_t)(sum / 4);
	}
	return (uint8_t)(sum / 2);
}

bool Maker::flushReadyRows() {
	while (true) {
		uint8_t rowsNeeded = softDownsample_ ? 2 : 1;

		if (nextRowToWrite_ + rowsNeeded > decH_) return true;

		RowCache* r0 = &rows_[nextRowToWrite_ % cfg_.cacheRows];
		if (r0->y != (int32_t)nextRowToWrite_ || r0->filledCount != decW_) return true;

		RowCache* r1 = softDownsample_ ? &rows_[(nextRowToWrite_ + 1) % cfg_.cacheRows] : nullptr;
		if (softDownsample_) {
			if (r1->y != (int32_t)(nextRowToWrite_ + 1) || r1->filledCount != decW_) return true;
		}

		// quando softDownsample, você está reduzindo 2x2 => passo maior
		uint8_t xStep = softDownsample_ ? 4 : 2;

		for (uint16_t x = 0; x < decW_; x += xStep) {
			uint8_t p1 = softDownsample_ ? getAverage2x2_4bit(r0, r1, x) : r0->data[x];

			uint8_t p2 = 0;
			uint16_t nextX = x + (softDownsample_ ? 2 : 1);
			if (nextX < decW_) {
				p2 = softDownsample_ ? getAverage2x2_4bit(r0, r1, nextX) : r0->data[nextX];
			}

			outFile_.write((uint8_t)((p1 << 4) | (p2 & 0x0F)));
		}

		// limpa linhas consumidas
		resetRow(nextRowToWrite_);
		if (r1) resetRow(nextRowToWrite_ + 1);

		nextRowToWrite_ += rowsNeeded;
	}
}

bool Maker::tjpgOutputCb(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
	if (!active_) return false;
	return active_->handleTjpgBlock(x, y, w, h, bitmap);
}

bool Maker::handleTjpgBlock(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
	for (uint16_t row = 0; row < h; row++) {
		uint16_t curY = (uint16_t)(y + row);
		if (curY >= decH_) continue;

		RowCache* r = &rows_[curY % cfg_.cacheRows];
		if (r->y != (int32_t)curY) {
			r->y = curY;
			r->filledCount = 0;
			memset(r->filled, 0, filledBytes_);
		}

		for (uint16_t col = 0; col < w; col++) {
			uint16_t curX = (uint16_t)(x + col);
			if (curX < decW_ && !bitGetAt(r->filled, curX)) {
				r->data[curX] = rgb565ToGray4(bitmap[row * w + col]);
				bitSetAt(r->filled, curX);
				r->filledCount++;
			}
		}
	}
	return flushReadyRows();
}

bool Maker::makeThumbnail(const String& path, const String& fileName) {
	uint16_t w = 0, h = 0;
	if (TJpgDec.getFsJpgSize(&w, &h, path.c_str(), SD) != JDR_OK) return false;

	// Decide jpgScale_ (downscale “hardware” do TJpgDec)
	jpgScale_ = 1;
	if (w / 8 >= cfg_.targetWidth) jpgScale_ = 8;
	else if (w / 4 >= cfg_.targetWidth) jpgScale_ = 4;
	else if (w / 2 >= cfg_.targetWidth) jpgScale_ = 2;

	uint16_t currentW = w / jpgScale_;
	if (currentW > cfg_.targetWidth + 20) {
		softDownsample_ = true;
		fileW_ = currentW / 2;
		fileH_ = (h / jpgScale_) / 2;
	} else {
		softDownsample_ = false;
		fileW_ = currentW;
		fileH_ = h / jpgScale_;
	}

	if (cfg_.verbose) {
		Serial.println("\n--- Iniciando Conversao Otimizada ---");
		Serial.printf("Origem: %s (%dx%d)\n", path.c_str(), w, h);
		Serial.printf("Escala HW: %u | SoftDown: %s\n", jpgScale_, softDownsample_ ? "SIM" : "NAO");
		Serial.printf("Saida Final: %ux%u\n", fileW_, fileH_);
	}

	TJpgDec.setJpgScale(jpgScale_);
	TJpgDec.setCallback(Maker::tjpgOutputCb);

	decW_ = w / jpgScale_;
	decH_ = h / jpgScale_;

	if (!ensureRowCacheAllocated(decW_)) return false;

	// Monta output .bin
	String base = fileName;
	int dot = base.lastIndexOf('.');
	if (dot > 0) base = base.substring(0, dot);

	String outPath = String(cfg_.outDir) + "/" + base + ".bin";
	SD.remove(outPath);

	outFile_ = SD.open(outPath, FILE_WRITE);
	if (!outFile_) return false;

	// Header (little-endian) W/H do arquivo final
	outFile_.write((uint8_t)(fileW_ & 0xFF));
	outFile_.write((uint8_t)(fileW_ >> 8));
	outFile_.write((uint8_t)(fileH_ & 0xFF));
	outFile_.write((uint8_t)(fileH_ >> 8));

	nextRowToWrite_ = 0;

	// Ativa objeto para callback
	active_ = this;
	TJpgDec.drawFsJpg(0, 0, path.c_str(), SD);
	active_ = nullptr;

	outFile_.close();

	uint32_t finalSize = 0;
	File checkFile = SD.open(outPath, FILE_READ);
	if (checkFile) { finalSize = checkFile.size(); checkFile.close(); }

	if (cfg_.verbose) {
		Serial.printf(">>> SUCESSO! Arquivo: %s (%lu bytes)\n", outPath.c_str(), (unsigned long)finalSize);
	}

	return true;
}

void Maker::processWaitlist() {
	File dir = SD.open(cfg_.srcDir);
	if (!dir || !dir.isDirectory()) {
		Serial.printf("Erro: pasta %s nao encontrada!\n", cfg_.srcDir);
		return;
	}

	while (true) {
		File entry = dir.openNextFile();
		if (!entry) break;

		if (!entry.isDirectory() && endsWithJpg(entry.name())) {
			String fileName = String(entry.name());
			String fullPath = String(cfg_.srcDir) + "/" + fileName;
			String fullPathSent = String(cfg_.sentDir) + "/" + fileName;

			entry.close();

			if (makeThumbnail(fullPath, fileName)) {
				SD.rename(fullPath, fullPathSent);
				//SD.remove(fullPath);
				if (cfg_.verbose) {
					Serial.printf("Removido da WAITLIST e arquivado em DCIM_sent: %s\n", fileName.c_str());
				}
			} else {
				Serial.printf("Falha ao processar: %s\n", fullPath.c_str());
			}
		} else {
			entry.close();
		}
	}

	dir.close();
	if (cfg_.verbose) Serial.println("Fila WAITLIST vazia.");
}

} // namespace ThumbMaker
