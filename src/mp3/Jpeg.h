#include "ff.h"
#include "project.h"
#include "jpeg_utils.h"

constexpr uint32_t IN_BUFFER_SIZE = 4094; // 4kB SD to jpeg hw buffer
constexpr uint32_t OUT_BUFFER_SIZE = 768; // JPEG hw output buffer, 1 to 4 MCUs depending on color space & chroma sampling


typedef struct InputBuffer_t {
	uint32_t numBytes;
	uint8_t buffer[IN_BUFFER_SIZE];

} InputBuffer_t;

typedef struct OutputBuffer_t {
	uint32_t numBytes;
	uint8_t buffer[OUT_BUFFER_SIZE];
} OutputBuffer_t;

typedef struct JPEG_Data_Info_t {
	FIL fp;
	uint64_t startPos;
	uint64_t currPos;
	uint32_t size;
	uint32_t nbMcu;
	bool jpegDecoded;
	InputBuffer_t * currInputBuffer;
	JPEG_YCbCrToRGB_Convert_Function convertFunction;
} JPEG_Data_Info_t;

JPEG_YCbCrToRGB_Convert_Function convertFunction;

bool getJpegInfos();
