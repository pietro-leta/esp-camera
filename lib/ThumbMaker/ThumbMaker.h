#pragma once
#include <Arduino.h>
#include <FS.h>

// Encapsula tudo dentro de um namespace pra não poluir o projeto
namespace ThumbMaker {

struct Config {
	// SD / SPI
	int sdCs = 5;
	uint32_t spiFreq = 16000000;

	// Pastas
	const char* srcDir = "/WAITLIST";
	const char* outDir = "/THUMB";

	// Thumbnail target
	uint16_t targetWidth = 180;

	// Cache (linhas) - mexa aqui se quiser
	uint16_t cacheRows = 48;

	// Log
	bool verbose = false;
};

class Maker {
public:
	explicit Maker(const Config& cfg);

	// Inicializa SD e cria pasta de saída, etc.
	bool begin();

	// Processa a fila inteira: converte jpg/jpeg em .bin e remove da WAITLIST se ok
	void processWaitlist();

	// Converte um arquivo específico
	bool makeThumbnail(const String& fullPath, const String& fileName);

private:
	Config cfg_;

	// --- helpers de arquivos ---
	static bool endsWithJpg(const String& name);

	// --- conversão imagem / 4-bit ---
	static inline uint8_t rgb565ToGray4(uint16_t c);

	// --- bitset helpers ---
	static inline bool bitGetAt(const uint8_t* bits, uint16_t i);
	static inline void bitSetAt(uint8_t* bits, uint16_t i);

	// --- cache de linhas ---
	struct RowCache {
		uint8_t* data = nullptr;     // [decW]
		uint8_t* filled = nullptr;   // bitset [decW]
		uint16_t filledCount = 0;
		int32_t y = -1;
	};

	// Estado “global” do decoder (mantido dentro do objeto)
	uint8_t  jpgScale_ = 1;
	bool     softDownsample_ = false;

	uint16_t decW_ = 0, decH_ = 0;
	uint16_t fileW_ = 0, fileH_ = 0;

	File outFile_;
	uint16_t nextRowToWrite_ = 0;

	RowCache* rows_ = nullptr;
	uint16_t filledBytes_ = 0;

	// --- output TJpg_Decoder (precisa ser static) ---
	static Maker* active_;                 // ponteiro pro objeto ativo (callback)
	static bool tjpgOutputCb(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);

	// --- lógica interna ---
	uint8_t getAverage2x2_4bit(RowCache* r0, RowCache* r1, uint16_t x);
	bool flushReadyRows();
	bool handleTjpgBlock(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);

	bool ensureRowCacheAllocated(uint16_t newDecW);
	void resetRow(uint16_t y);
	void freeCaches();
};

} // namespace ThumbMaker
