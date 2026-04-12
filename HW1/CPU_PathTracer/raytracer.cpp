#include "include/Freeimage.h"
#include <string>
#include <cstdlib> // for malloc and free

const int IMAGE_WIDTH = 240;
const int IMAGE_HEIGHT = 120;

int main()
{
	std::string fname = "outfile.png";
	FreeImage_Initialise();

	BYTE* pixels = (BYTE*)malloc(sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);
	if (!pixels) {
		FreeImage_DeInitialise();
		return 1; // handle allocation failure
	}
	memset(pixels, 0, sizeof(BYTE) * IMAGE_WIDTH * IMAGE_HEIGHT * 3);

	for (int y = 0; y < IMAGE_HEIGHT; y++)
	{
		for (int x = 0; x < IMAGE_WIDTH; x++)
		{
			int idx = (y * IMAGE_WIDTH + x) * 3;
			pixels[idx + 0] = 255;
			pixels[idx + 1] = 255;
			pixels[idx + 2] = 255;
		}

	}
	FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);

	FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

	free(pixels);
	FreeImage_DeInitialise();
}