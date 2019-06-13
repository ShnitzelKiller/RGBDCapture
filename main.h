#pragma once
const int width = 512;
const int height = 424;

const int cwidth = 1920;
const int cheight = 1080;

void getIntrinsics(int value);
void drawKinectData();
void keycallback(unsigned char key, int x, int y);
void timer(int value);