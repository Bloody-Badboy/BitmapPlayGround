package com.bloodybadboycreation.bitmapplayground

/**
 * Created by bloodybadboycreation on 15/1/18.
 */
val R2 = Math.sqrt(2.0).toFloat()

val ROBERTS_V = arrayOf(floatArrayOf(0f, 0f, -1f), floatArrayOf(0f, 1f, 0f), floatArrayOf(0f, 0f, 0f))
val ROBERTS_H = arrayOf(floatArrayOf(-1f, 0f, 0f), floatArrayOf(0f, 1f, 0f), floatArrayOf(0f, 0f, 0f))

val PREWITT_V = arrayOf(floatArrayOf(-1f, 0f, 1f), floatArrayOf(-1f, 0f, 1f), floatArrayOf(-1f, 0f, 1f))
val PREWITT_H = arrayOf(floatArrayOf(-1f, -1f, -1f), floatArrayOf(0f, 0f, 0f), floatArrayOf(1f, 1f, 1f))

val SOBEL_V = arrayOf(floatArrayOf(-1f, 0f, 1f), floatArrayOf(-2f, 0f, 2f), floatArrayOf(-1f, 0f, 1f))
val SOBEL_H = arrayOf(floatArrayOf(-1f, -2f, -1f), floatArrayOf(0f, 0f, 0f), floatArrayOf(1f, 2f, 1f))

val FREI_CHEN_H = arrayOf(floatArrayOf(-1f, -R2, -1f), floatArrayOf(0f, 0f, 0f), floatArrayOf(1f, R2, 1f))
val FREI_CHEN_V = arrayOf(floatArrayOf(-1f, 0f, 1f), floatArrayOf(-R2, 0f, R2), floatArrayOf(-1f, 0f, 1f))
