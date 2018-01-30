#include <jni.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <android/log.h>
#include <android/bitmap.h>
#include <sys/time.h>

#define TAG "BBC_ImageProcessor"
#define LOGE(...) (__android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__))
#define LOGD(...) (__android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__))

// #define GetRVal(color) ((color >> 16) & 0xFF)
// #define GetGVal(color) ((color >> 8) & 0xFF)
// #define GetBVal(color) (color & 0xFF)
// #define ARGBToColor(a, r, g, b) ((a & 0xFF000000) | ((r << 16) & 0x00FF0000) | ((g << 8) & 0x0000FF00) | (b & 0x000000FF))

static long getCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static __inline__ jint *getPixelArray(JNIEnv *env, jintArray buff) {
    jint *pixelsBuff = (*env)->GetIntArrayElements(env, buff, 0);
    if (pixelsBuff == NULL) {
        LOGE("can't get pixels from buffer");
    }
    return pixelsBuff;
}

static __inline__ jintArray jintPointerToJintArray(JNIEnv *env, jint size, jint *arr) {
    jintArray result = (*env)->NewIntArray(env, size);
    (*env)->SetIntArrayRegion(env, result, 0, size, arr);
    return result;
}

static __inline__ void releasePixelsArray(JNIEnv *env, jintArray array1, jint *array2) {
    (*env)->ReleaseIntArrayElements(env, array1, array2, 0);
}

static float **get2dMatrixArray(JNIEnv *env, jobjectArray matrix) {
    int row = (*env)->GetArrayLength(env, matrix);
    jfloatArray dim = (jfloatArray) (*env)->GetObjectArrayElement(env, matrix, 0);
    int col = (*env)->GetArrayLength(env, dim);

    float **matrixArray;

    // allocate matrixArray using row
    matrixArray = malloc(sizeof(float *) * row);
    for (int i = 0; i < row; ++i) {
        jfloatArray oneDim = (jfloatArray) (*env)->GetObjectArrayElement(env, matrix, i);
        jfloat *elements = (*env)->GetFloatArrayElements(env, oneDim, 0);
        //allocate matrixArray[i] using col
        matrixArray[i] = malloc(sizeof(float) * col);
        for (int j = 0; j < col; ++j) {
            matrixArray[i][j] = elements[j];
        }
    }
    return matrixArray;
}

static void release2dMatrixArray(JNIEnv *env, jobjectArray matrix) {
    int row = (*env)->GetArrayLength(env, matrix);
    for (int i = 0; i < row; i++) {
        jfloatArray oneDim = (jfloatArray) (*env)->GetObjectArrayElement(env, matrix, i);
        jfloat *elements = (*env)->GetFloatArrayElements(env, oneDim, 0);

        (*env)->ReleaseFloatArrayElements(env, oneDim, elements, 0);
        (*env)->DeleteLocalRef(env, oneDim);
    }
}

static void hueToRgb(unsigned int *c, float p, float q, float t) {
    if (t < 0.0f)
        t += 1.0f;
    if (t > 1.0f)
        t -= 1.0f;
    if (t < 0.166666)
        *c = (unsigned int) ((q + (p - q) * 6 * t) * 100);
    else if (t < 0.5)
        *c = (unsigned int) (p * 100);
    else if (t < 0.666666)
        *c = (unsigned int) ((q + (p - q) * (.666666 - t) * 6) * 100);
    else
        *c = (unsigned int) (q * 100);
}

static void greyScale(int *pixels, int width, int height) {

    int red, green, blue, rgb;

    for (int i = 0; i < width * height; i++) {

        red = (pixels[i] >> 16) & 0xFF;
        green = (pixels[i] >> 8) & 0xFF;
        blue = (pixels[i]) & 0xFF;

        //  http://www.jhlabs.com/ip/filters/GrayscaleFilter.html
        rgb = (red * 77 + green * 151 + blue * 28) >> 8;    // NTSC luma

        pixels[i] = (pixels[i] & 0xFF000000) | ((rgb << 16) & 0x00FF0000) |
                    ((rgb << 8) & 0x0000FF00) | (rgb & 0x000000FF);
    }
}

static void colorOverlay(int *pixels, int width, int height, int depth, float red, float green, float blue) {
    int R, G, B;

    for (int i = 0; i < width * height; i++) {

        R = (pixels[i] >> 16) & 0xFF;
        G = (pixels[i] >> 8) & 0xFF;
        B = (pixels[i]) & 0xFF;

        R += (depth * red);
        G += (depth * green);
        B += (depth * blue);

        if (R > 255)
            R = 255;

        if (G > 255)
            G = 255;

        if (B > 255)
            B = 255;

        pixels[i] = (pixels[i] & 0xFF000000) | ((R << 16) & 0x00FF0000) |
                    ((G << 8) & 0x0000FF00) | (B & 0x000000FF);
    }

}

static void colorBoost(int *pixels, int width, int height, float redPercent, float greenPercent, float bluePercent) {

    int red, green, blue;

    for (int i = 0; i < width * height; i++) {

        red = (pixels[i] >> 16) & 0xFF;
        green = (pixels[i] >> 8) & 0xFF;
        blue = (pixels[i]) & 0xFF;


        red *= (1 + redPercent);
        green *= (1 + greenPercent);
        blue *= (1 + bluePercent);

        pixels[i] = (pixels[i] & 0xFF000000) | ((red << 16) & 0x00FF0000) |
                    ((green << 8) & 0x0000FF00) | (blue & 0x000000FF);
    }

}

static void brightness(int *pixels, int width, int height, int level) {

    int red, green, blue;

    for (int i = 0; i < width * height; i++) {

        red = (pixels[i] >> 16) & 0xFF;
        green = (pixels[i] >> 8) & 0xFF;
        blue = (pixels[i]) & 0xFF;

        red += level;
        green += level;
        blue += level;

        if (red > 255)
            red = 255;
        else if (red < 0)
            red = 0;

        if (green > 255)
            green = 255;
        else if (green < 0)
            green = 0;

        if (blue > 255)
            blue = 255;
        else if (blue < 0)
            blue = 0;

        pixels[i] = (pixels[i] & 0xFF000000) | ((red << 16) & 0x00FF0000) |
                    ((green << 8) & 0x0000FF00) | (blue & 0x000000FF);
    }
}

static void contrast(int *pixels, int width, int height, float level) {

    int red, green, blue;

    for (int i = 0; i < width * height; i++) {

        red = (pixels[i] >> 16) & 0xFF;
        green = (pixels[i] >> 8) & 0xFF;
        blue = (pixels[i]) & 0xFF;

        red = (int) (((((red / 255.0f) - 0.5f) * level) + 0.5f) * 255.0f);
        green = (int) (((((green / 255.0f) - 0.5f) * level) + 0.5f) * 255.0f);
        blue = (int) (((((blue / 255.0f) - 0.5f) * level) + 0.5f) * 255.0f);

        if (red > 255)
            red = 255;
        else if (red < 0)
            red = 0;

        if (green > 255)
            green = 255;
        else if (green < 0)
            green = 0;

        if (blue > 255)
            blue = 255;
        else if (blue < 0)
            blue = 0;

        pixels[i] = (pixels[i] & 0xFF000000) | ((red << 16) & 0x00FF0000) |
                    ((green << 8) & 0x0000FF00) | (blue & 0x000000FF);
    }
}

static void gamma(int *pixels, int width, int height, float redPercent, float greenPercent, float bluePercent) {
    int rTable[256], gTable[256], bTable[256];

    for (int i = 0; i < 256; ++i) {
        rTable[i] = (int) ((255.0 * pow(i / 255.0, 1.0 / redPercent)) + 0.5);
        gTable[i] = (int) ((255.0 * pow(i / 255.0, 1.0 / greenPercent)) + 0.5);
        bTable[i] = (int) ((255.0 * pow(i / 255.0, 1.0 / bluePercent)) + 0.5);

        if (rTable[i] > 255)
            rTable[i] = 255;
        if (gTable[i] > 255)
            gTable[i] = 255;
        if (bTable[i] > 255)
            bTable[i] = 255;
    }

    for (int i = 0; i < width * height; i++) {

        pixels[i] = (pixels[i] & 0xFF000000) | ((rTable[(pixels[i] >> 16) & 0xFF] << 16) & 0x00FF0000) |
                    ((gTable[(pixels[i] >> 8) & 0xFF] << 8) & 0x0000FF00) | (bTable[pixels[i] & 0xFF] & 0x000000FF);
    }

}

static void invert(int *pixels, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        pixels[i] = (pixels[i] & 0xFF000000) | (~pixels[i] & 0x00FFFFFF);
    }

}

static void saturation(int *pixels, int width, int height, float level) {
    unsigned int R, G, B;
    float red, green, blue;
    float max, min, delta;
    float H, S, L;
    float p, q;

    // https://gist.github.com/mjackson/5311256

    for (int i = 0; i < width * height; i++) {

        R = (unsigned int) ((pixels[i] >> 16) & 0xFF);
        G = (unsigned int) ((pixels[i] >> 8) & 0xFF);
        B = (unsigned int) ((pixels[i]) & 0xFF);


        red = (float) R / 255.0f;
        green = (float) G / 255.0f;
        blue = (float) B / 255.0f;

        max = red > (green > blue ? green : blue) ? red : green > blue ? green : blue;
        min = red < (green < blue ? green : blue) ? red : green < blue ? green : blue;

        H = S = L = 0;
        L = (max + min) / 2;

        if (max != min) {
            delta = max - min;

            S = delta / (L < .50 ? max + min : 2 - max - min);

            if (max == red) {
                H = (green - blue) / (delta);
            } else if (max == green) {
                H = 2 + (blue - red) / delta;
            } else if (max == blue) {
                H = 4 + (red - green) / delta;
            }
            H /= 6;
        }

        S *= 100;
        S *= level;
        S = S > 100 ? 100 : S < 0 ? 0 : S;

        S = S / 100;

        if (S == 0) {
            R = G = B = (unsigned int) (L * 100);
        } else {
            p = L < .50 ? L * (1 + S) : L + S - (L * S);
            q = 2 * L - p;


            hueToRgb(&R, p, q, H + .33333f);

            hueToRgb(&G, p, q, H);

            hueToRgb(&B, p, q, H - .33333f);


        }
        R = (unsigned int) (((((float) R) / 100) * 255) + 0.5);
        G = (unsigned int) (((((float) G) / 100) * 255) + 0.5);
        B = (unsigned int) (((((float) B) / 100) * 255) + 0.5);

        pixels[i] = (pixels[i] & 0xFF000000) | ((R << 16) & 0x00FF0000) |
                    ((G << 8) & 0x0000FF00) | (B & 0x000000FF);

    }
}


static void convolution(int *inPixels, int width, int height, float **matrixArray, float divisor, float offset) {
    int red, green, blue, alpha;
    int rSum, gSum, bSum;
    int y, x, i, j, iy, ix;

    int *outPixels = malloc(width * height * sizeof(int));

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {

            alpha = inPixels[y * width + x] & 0xFF000000;
            rSum = gSum = bSum = 0;

            for (i = -1; i <= 1; i++) {
                iy = y + i;
                if (!(iy >= 0 && iy < height))
                    iy = y;
                for (j = -1; j <= 1; j++) {
                    ix = x + j;
                    if (!(ix >= 0 && ix < width))
                        ix = x;

                    rSum += (int) (((inPixels[iy * width + ix] >> 16) & 0xFF) * matrixArray[i + 1][j + 1]);
                    gSum += (int) (((inPixels[iy * width + ix] >> 8) & 0xFF) * matrixArray[i + 1][j + 1]);
                    bSum += (int) ((inPixels[iy * width + ix] & 0xFF) * matrixArray[i + 1][j + 1]);
                }
            }


            red = (int) (rSum / divisor + offset);
            green = (int) (gSum / divisor + offset);
            blue = (int) (bSum / divisor + offset);


            if (red > 255)
                red = 255;
            else if (red < 0)
                red = 0;

            if (green > 255)
                green = 255;
            else if (green < 0)
                green = 0;

            if (blue > 255)
                blue = 255;
            else if (blue < 0)
                blue = 0;

            outPixels[y * width + x] = (alpha & 0xFF000000) | ((red << 16) & 0x00FF0000) |
                                       ((green << 8) & 0x0000FF00) | (blue & 0x000000FF);

        }
    }

    memmove(inPixels, outPixels, width * height * sizeof(uint32_t));
}

void edgeDetect(int *inPixels, int width, int height, float **vEdgeMatrixArray, float **hEdgeMatrixArray) {
    int alpha, red, green, blue;
    int rSumH, gSumH, bSumH;
    int rSumV, gSumV, bSumV;


    uint32_t *outPixels = malloc(width * height * sizeof(uint32_t));

    for (int y = 0; y < height; ++y) {

        for (int x = 0; x < width; ++x) {

            rSumH = gSumH = bSumH = 0;
            rSumV = gSumV = bSumV = 0;

            alpha = inPixels[y * width + x] & 0xFF000000;

            for (int row = -1; row <= 1; row++) {
                int iy = y + row;
                int ioffset;

                if (iy >= 0 && iy < height) {
                    ioffset = iy * width;
                } else {
                    ioffset = y * width;
                }

                for (int col = -1; col <= 1; col++) {
                    int ix = x + col;

                    if (!(ix >= 0 && ix < width)) {
                        ix = x;
                    }

                    red = (inPixels[ioffset + ix] >> 16) & 0xFF;
                    green = (inPixels[ioffset + ix] >> 8) & 0xFF;
                    blue = inPixels[ioffset + ix] & 0xFF;


                    rSumV += (int) (red * vEdgeMatrixArray[row + 1][col + 1]);
                    gSumV += (int) (green * vEdgeMatrixArray[row + 1][col + 1]);
                    bSumV += (int) (blue * vEdgeMatrixArray[row + 1][col + 1]);

                    rSumH += (int) (red * hEdgeMatrixArray[row + 1][col + 1]);
                    gSumH += (int) (green * hEdgeMatrixArray[row + 1][col + 1]);
                    bSumH += (int) (blue * hEdgeMatrixArray[row + 1][col + 1]);
                }
            }

            red = (int) (sqrt(rSumH * rSumH + rSumV * rSumV) / 1.8f);
            green = (int) (sqrt(gSumH * gSumH + gSumV * gSumV) / 1.8f);
            blue = (int) (sqrt(bSumH * bSumH + bSumV * bSumV) / 1.8f);


            // clamp rgb values
            if (red > 255)
                red = 255;
            else if (red < 0)
                red = 0;

            if (green > 255)
                green = 255;
            else if (green < 0)
                green = 0;

            if (blue > 255)
                blue = 255;
            else if (blue < 0)
                blue = 0;


            outPixels[x + width * y] = (alpha & 0xFF000000) | ((red << 16) & 0x00FF0000) |
                                       ((green << 8) & 0x0000FF00) | (blue & 0x000000FF);
        }
    }

    memmove(inPixels, outPixels, (width * height * sizeof(uint32_t)));

}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doBrightness(JNIEnv *env,
                                                                                                       jobject instance,
                                                                                                       jintArray pixels_,
                                                                                                       jint width,
                                                                                                       jint height,
                                                                                                       jint level) {
    jint *pixels = getPixelArray(env, pixels_);


    long startTime = getCurrentTime();
    brightness(pixels, width, height, level);
    long endTime = getCurrentTime();

    LOGD("doBrightness() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doInvert(JNIEnv *env, jclass type,
                                                                                                   jintArray pixels_,
                                                                                                   jint width, jint height) {
    jint *pixels = getPixelArray(env, pixels_);

    invert(pixels, width, height);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}


JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doSaturation(JNIEnv *env, jclass type,
                                                                                                       jintArray pixels_,
                                                                                                       jint width, jint height,
                                                                                                       jfloat level) {
    jint *pixels = getPixelArray(env, pixels_);


    long startTime = getCurrentTime();
    saturation(pixels, width, height, level);
    long endTime = getCurrentTime();

    LOGD("doSaturation() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}


JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doColorBoost(JNIEnv *env, jclass type,
                                                                                                       jintArray pixels_,
                                                                                                       jint width, jint height,
                                                                                                       jfloat redPercent,
                                                                                                       jfloat greenPercent,
                                                                                                       jfloat bluePercent) {
    jint *pixels = getPixelArray(env, pixels_);

    long startTime = getCurrentTime();
    colorBoost(pixels, width, height, redPercent, greenPercent, bluePercent);
    long endTime = getCurrentTime();

    LOGD("doColorBoost() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doContrast(JNIEnv *env, jclass type,
                                                                                                     jintArray pixels_,
                                                                                                     jint width, jint height,
                                                                                                     jfloat level) {
    jint *pixels = getPixelArray(env, pixels_);

    long startTime = getCurrentTime();
    contrast(pixels, width, height, level);
    long endTime = getCurrentTime();

    LOGD("doContrast() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doColorOverlay(JNIEnv *env,
                                                                                                         jclass type,
                                                                                                         jintArray pixels_,
                                                                                                         jint width,
                                                                                                         jint height,
                                                                                                         jint depth, jfloat red,
                                                                                                         jfloat green,
                                                                                                         jfloat blue) {
    jint *pixels = getPixelArray(env, pixels_);

    long startTime = getCurrentTime();
    colorOverlay(pixels, width, height, depth, red, green, blue);
    long endTime = getCurrentTime();

    LOGD("doColorOverlay() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doGreyScale(JNIEnv *env, jclass type,
                                                                                                      jintArray pixels_,
                                                                                                      jint width, jint height) {
    jint *pixels = getPixelArray(env, pixels_);

    long startTime = getCurrentTime();
    greyScale(pixels, width, height);
    long endTime = getCurrentTime();

    LOGD("doGreyScale() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;

}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doGamma(JNIEnv *env, jclass type,
                                                                                                  jintArray pixels_,
                                                                                                  jint width, jint height,
                                                                                                  jfloat redPercent,
                                                                                                  jfloat greenPercent,
                                                                                                  jfloat bluePercent) {
    jint *pixels = getPixelArray(env, pixels_);

    long startTime = getCurrentTime();
    gamma(pixels, width, height, redPercent, greenPercent, bluePercent);
    long endTime = getCurrentTime();

    LOGD("doGamma() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixels);
    releasePixelsArray(env, pixels_, pixels);
    return result;
}

JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doConvolution(JNIEnv *env, jclass type,
                                                                                                        jintArray pixelsIn_,
                                                                                                        jint width, jint height,
                                                                                                        jobjectArray matrix,
                                                                                                        jfloat divisor,
                                                                                                        jfloat offset) {
    jint *pixelsIn = getPixelArray(env, pixelsIn_);

    long startTime = getCurrentTime();

    float **matrixArray = get2dMatrixArray(env, matrix);

    convolution(pixelsIn, width, height, matrixArray, divisor, offset);

    long endTime = getCurrentTime();
    LOGD("doConvolution() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixelsIn);

    release2dMatrixArray(env, matrix);
    releasePixelsArray(env, pixelsIn_, pixelsIn);
    return result;
}


JNIEXPORT jintArray JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doEdgeDetect(JNIEnv *env, jclass type,
                                                                                                       jintArray pixelsIn_, jint width, jint height,
                                                                                                       jobjectArray vEdgeMatrix,
                                                                                                       jobjectArray hEdgeMatrix) {
    jint *pixelsIn = getPixelArray(env, pixelsIn_);

    long startTime = getCurrentTime();

    float **vEdgeMatrixArray = get2dMatrixArray(env, vEdgeMatrix);
    float **hEdgeMatrixArray = get2dMatrixArray(env, hEdgeMatrix);

    edgeDetect(pixelsIn, width, height, vEdgeMatrixArray, hEdgeMatrixArray);

    long endTime = getCurrentTime();
    LOGD("doEdgeDetect() -> %ld ms", endTime - startTime);

    jintArray result = jintPointerToJintArray(env, width * height, pixelsIn);

    release2dMatrixArray(env, vEdgeMatrix);
    release2dMatrixArray(env, hEdgeMatrix);

    releasePixelsArray(env, pixelsIn_, pixelsIn);

    return result;
}

JNIEXPORT void JNICALL Java_com_bloodybadboycreation_bitmapplayground_ImageProcessor_doPlayWithBitmap(JNIEnv *env, jclass type,
                                                                                                      jobject bitmap, jintArray inkPixels_) {
    AndroidBitmapInfo bitmapInfo;
    int resultCode;
    void *pixels1, *pixles2;

    long startTime = getCurrentTime();

    if ((resultCode = AndroidBitmap_getInfo(env, bitmap, &bitmapInfo)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed! error=%d", resultCode);
        return;
    }

    if (bitmapInfo.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
        LOGE("Bitmap format is not RGBA_8888");
        return;
    }

    if ((resultCode = AndroidBitmap_lockPixels(env, bitmap, &pixels1)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed! error=%d", resultCode);
    }


    uint32_t alpha, red, green, blue;
    uint32_t alpha1, red1, green1, blue1;
    uint32_t R, G, B;

    uint32_t pixelsCount = bitmapInfo.width * bitmapInfo.height;
    uint32_t *inPixels = pixels1;

    jint *inkPixels = getPixelArray(env, inkPixels_);

    // memcpy(tmpPixels, inPixels, pixelsCount * sizeof(uint32_t));


    for (int i = 0; i < pixelsCount; i++) {
        red = (inPixels[i] >> 16) & 0xFF;
        green = (inPixels[i] >> 8) & 0xFF;
        blue = (inPixels[i]) & 0xFF;

        alpha1 = (uint32_t) (inkPixels[i] >> 24 & 0xFF);

        red1 = (uint32_t) ((inkPixels[i] >> 16) & 0xFF);
        green1 = (uint32_t) ((inkPixels[i] >> 8) & 0xFF);
        blue1 = (uint32_t) ((inkPixels[i]) & 0xFF);


        if (red1 <= 0) {
            red1 = 1;
        }
        if (green1 <= 0) {
            green1 = 1;
        }

        if (blue1 <= 0) {
            blue1 = 1;
        }

        R = (uint32_t) (red + red1);

        G = (uint32_t) (green +green1);

        B = (uint32_t) (blue +  blue1);


        if (R > 255)
            R = 255;

        if (G > 255)
            G = 255;

        if (B > 255)
            B = 255;

        red = R;
        blue = B;
        green = G;


        inPixels[i] = (inPixels[i] & 0xFF000000) | ((red << 16) & 0x00FF0000) | ((green << 8) & 0x0000FF00) | (blue & 0x000000FF);
    }

    if ((resultCode = AndroidBitmap_unlockPixels(env, bitmap)) < 0) {
        LOGE("AndroidBitmap_unlockPixels() failed! error=%d", resultCode);
    }

    releasePixelsArray(env, inkPixels_, inkPixels);

    long endTime = getCurrentTime();
    LOGD("doPlayWithBitmap() -> %ld ms", endTime - startTime);

}
