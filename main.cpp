#include "main.h"
#include "glut.h"

#include <Windows.h>
#include <Ole2.h>

#include <Kinect.h>

#include <string>
#include <iostream>
#include <fstream>
#define maxdepth 1500

// OpenGL Variables
GLuint textureId;
GLubyte data[width*height*4];
GLubyte cdata[cwidth*cheight*4];

char colorImage[cwidth*cheight * 3];
char depthImage[width*height * 3];

int picturecount=0;

// Kinect variables
IKinectSensor* sensor;         // Kinect sensor
IDepthFrameReader* reader;     // Kinect depth data source
IColorFrameReader* creader;    // Kinect color data source
CameraIntrinsics depth_intrinsics;

bool initKinect() {
    if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->Open();
		IDepthFrameSource* framesource = NULL;
		sensor->get_DepthFrameSource(&framesource);
		framesource->OpenReader(&reader);
		if (framesource) {
			framesource->Release();
			framesource = NULL;
		}
		IColorFrameSource* cframesource = NULL;
		sensor->get_ColorFrameSource(&cframesource);
		cframesource->OpenReader(&creader);
		if (cframesource) {
			cframesource->Release();
			cframesource = NULL;
		}
		glutTimerFunc(1000, getIntrinsics, 0);
		return true;
	} else {
		return false;
	}
}

void getIntrinsics(int value) {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		std::cout << "getting sensor failed" << std::endl;
		return;
	}
	if (sensor) {
		sensor->Open();
		ICoordinateMapper* coordmapper = NULL;
			if (!SUCCEEDED(sensor->get_CoordinateMapper(&coordmapper))) {
				std::cout << "failed to get coordinatemapper" << std::endl;
			}
		if (!SUCCEEDED(coordmapper->GetDepthCameraIntrinsics(&depth_intrinsics))) {
			std::cout << "failed to get intrinsics" << std::endl;
		}
		std::cout << "fX" << std::to_string(depth_intrinsics.FocalLengthX) << std::endl;
		std::cout << "fY" << std::to_string(depth_intrinsics.FocalLengthY) << std::endl;
		std::cout << "pX" << std::to_string(depth_intrinsics.PrincipalPointX) << std::endl;
		std::cout << "pY" << std::to_string(depth_intrinsics.PrincipalPointY) << std::endl;
		std::cout << "K2" << std::to_string(depth_intrinsics.RadialDistortionSecondOrder) << std::endl;
		std::cout << "K4" << std::to_string(depth_intrinsics.RadialDistortionFourthOrder) << std::endl;
		std::cout << "K6" << std::to_string(depth_intrinsics.RadialDistortionSixthOrder) << std::endl;

		if (coordmapper) {
			coordmapper->Release();
			coordmapper = NULL;
		}
	}
	else {
		std::cout << "opening sensor failed" << std::endl;
	}
}

void getKinectData(GLubyte* dest, GLubyte* cdest) {
    IDepthFrame* frame = NULL;
	IColorFrame* cframe = NULL;
    if (SUCCEEDED(reader->AcquireLatestFrame(&frame)) && SUCCEEDED(creader->AcquireLatestFrame(&cframe))) {

		//copy RGB
		cframe->CopyConvertedFrameDataToArray(cwidth*cheight * 4, cdest, ColorImageFormat_Bgra);

		//copy depth
        unsigned int sz;
		unsigned short* buf;
		frame->AccessUnderlyingBuffer(&sz, &buf);

		const unsigned short* curr = (const unsigned short*)buf;
		const unsigned short* dataEnd = curr + (width*height);

		while (curr < dataEnd) {
			// Get depth in millimeters
			unsigned short depth = (*curr++);

			// Draw a grayscale image of the depth:
			// B,G,R are all set to depth%256, alpha set to 1.
			unsigned int scaleddepth = ((unsigned int)(depth) * 255) / maxdepth;
			for (int i = 0; i < 3; ++i)
				*dest++ = (BYTE)(scaleddepth < 255 ? scaleddepth : 255);
			*dest++ = 0xff;
		}
    }
    if (frame) frame->Release();
	if (cframe) cframe->Release();
}

void drawKinectData() {
    glBindTexture(GL_TEXTURE_2D, textureId);
    getKinectData(data, cdata);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*) data);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glTexCoord2f(1.0f, 0.0f);
        glVertex3f(width, 0, 0);
        glTexCoord2f(1.0f, 1.0f);
        glVertex3f(width, height, 0.0f);
        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(0, height, 0.0f);
    glEnd();
}

bool savedata(std::string filename, GLubyte* sourceImageData, char* destImageData, int w, int h) {
	std::ofstream pFile(filename, std::ios_base::binary);
	if (!pFile.is_open()) {
		return false;
	}
	for (int i = 0; i < w*h; i++) {
		destImageData[(w*h-i) * 3] = sourceImageData[i * 4];
		destImageData[(w*h - i) * 3 + 1] = sourceImageData[i * 4 + 1];
		destImageData[(w*h - i) * 3 + 2] = sourceImageData[i * 4 + 2];
	}
	BITMAPINFOHEADER bmih;
	bmih.biSize = sizeof(BITMAPINFOHEADER);
	bmih.biWidth = w;
	bmih.biHeight = h;
	bmih.biPlanes = 1;
	bmih.biBitCount = 24;
	bmih.biCompression = BI_RGB;
	bmih.biSizeImage = w * h * 3;

	BITMAPFILEHEADER bmfh;
	int nBitsOffset = sizeof(BITMAPFILEHEADER) + bmih.biSize;
	LONG lImageSize = bmih.biSizeImage;
	LONG lFileSize = nBitsOffset + lImageSize;
	bmfh.bfType = 'B' + ('M' << 8);
	bmfh.bfOffBits = nBitsOffset;
	bmfh.bfSize = lFileSize;
	bmfh.bfReserved1 = bmfh.bfReserved2 = 0;

	// Write the bitmap file header
	pFile.write((const char*)&bmfh, sizeof(BITMAPFILEHEADER));
	UINT nWrittenFileHeaderSize = pFile.tellp();

	// And then the bitmap info header
	pFile.write((const char*)&bmih, sizeof(BITMAPINFOHEADER));
	UINT nWrittenInfoHeaderSize = pFile.tellp();

	// Finally, write the image data itself
	//-- the data represents our drawing
	pFile.write((char*)destImageData, w*h * 3);
	UINT nWrittenDIBDataSize = pFile.tellp();
	pFile.close();
}

void save() {
	picturecount++;
	std::cout << "saving " << picturecount << std::endl;
	// Create a new file for writing
	std::string cfilename = "saved_rgb\\RGB" + std::to_string(picturecount) + ".bmp";
	std::string dfilename = "saved_depth\\Depth" + std::to_string(picturecount) + ".bmp";
	savedata(cfilename, cdata, colorImage, cwidth, cheight);
	savedata(dfilename, data, depthImage, width, height);
}

void timer(int value) {
	save();
	glutTimerFunc(1000, timer, 0);
}
void keycallback(unsigned char key, int x, int y) {
	save();
}

int main(int argc, char* argv[]) {
	AllocConsole();
	freopen("CONIN$", "r", stdin);
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);
    if (!init(argc, argv)) return 1;
    if (!initKinect()) return 1;

    // Initialize textures
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*) data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // OpenGL setup
    glClearColor(0,0,0,0);
    glClearDepth(1.0f);
    glEnable(GL_TEXTURE_2D);

    // Camera setup
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Main loop
    execute();
    return 0;
}
