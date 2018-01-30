package com.bloodybadboycreation.bitmapplayground

import android.graphics.Bitmap

/**
 * Created by bloodybadboycreation on 4/12/17.
 */

object ImageProcessor {

    external fun doBrightness(pixels: IntArray, width: Int, height: Int, level: Int): IntArray

    external fun doContrast(pixels: IntArray, width: Int, height: Int, contrast: Float): IntArray

    external fun doInvert(pixels: IntArray, width: Int, height: Int): IntArray

    external fun doSaturation(pixels: IntArray, width: Int, height: Int, level: Float): IntArray

    external fun doGreyScale(pixels: IntArray, width: Int, height: Int): IntArray

    external fun doColorBoost(pixels: IntArray, width: Int, height: Int, redPercent: Float, greenPercent: Float, bluePercent: Float): IntArray

    external fun doColorOverlay(pixels: IntArray, width: Int, height: Int, depth: Int, red: Float, green: Float, blue: Float): IntArray

    external fun doGamma(pixels: IntArray, width: Int, height: Int, redPercent: Float, greenPercent: Float, bluePercent: Float): IntArray

    external fun doConvolution(pixels: IntArray, width: Int, height: Int, matrix: Array<FloatArray>, divisor: Float, offset: Float): IntArray

    external fun doEdgeDetect(pixels: IntArray, width: Int, height: Int, vEdgeMatrix: Array<FloatArray>, hEdgeMatrix: Array<FloatArray>)

    external fun doPlayWithBitmap(bitmap: Bitmap, inkBitmap: IntArray)
}
