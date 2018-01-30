package com.bloodybadboycreation.bitmapplayground

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Bundle
import android.support.v7.app.AppCompatActivity
import android.widget.ImageView


class MainActivity : AppCompatActivity() {


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val originalImageView = findViewById<ImageView>(R.id.original_iv)
        val processedImageView = findViewById<ImageView>(R.id.processed_iv)

        val scaledBitmap = decodeBitmapFromResource(R.drawable.girl, 1280, 800)
        val inkBitmap = decodeBitmapFromResource(R.drawable.overlay, 1280, 800)

        val width = scaledBitmap.width
        val height = scaledBitmap.height


        originalImageView.setImageBitmap(scaledBitmap)


        val inputPixels = IntArray(width * height)
        scaledBitmap.getPixels(inputPixels, 0, width, 0, 0, width, height)

        val inkBitmapPixels = IntArray(width * height)
        inkBitmap.getPixels(inkBitmapPixels, 0, inkBitmap.width, 0, 0, inkBitmap.width, inkBitmap.height)


        val outImage = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        outImage.setPixels(inputPixels, 0, width, 0, 0, width, height)


        ImageProcessor.doPlayWithBitmap(outImage, inkBitmapPixels)
        processedImageView.setImageBitmap(outImage)


    }

    private fun decodeBitmapFromResource(resId: Int, reqWidth: Int, reqHeight: Int): Bitmap {

        val options = BitmapFactory.Options()
        options.inJustDecodeBounds = true

        BitmapFactory.decodeResource(resources, resId, options)

        var width = options.outWidth
        var height = options.outHeight


        val reqRatio = reqWidth.toFloat() / reqHeight.toFloat()
        var imgRatio = width.toFloat() / height.toFloat()

        if (height > reqHeight || width > reqWidth) {
            when {
                imgRatio < reqRatio -> {
                    imgRatio = reqHeight.toFloat() / height.toFloat()
                    width = (imgRatio * width).toInt()
                    height = reqHeight
                }
                imgRatio > reqRatio -> {
                    imgRatio = reqWidth.toFloat() / width.toFloat()
                    height = (imgRatio * height).toInt()
                    width = reqWidth
                }
                else -> {
                    height = reqHeight
                    width = reqWidth

                }
            }
        }
        Log.d("Required Bitmap -> Width: $width, Height: $height")

        options.inSampleSize = calculateInSampleSize(options, width, height)
        options.inJustDecodeBounds = false

        Log.d("Decode Using -> inSampleSize: ${options.inSampleSize}")

        var scaledBitmap = BitmapFactory.decodeResource(resources, resId, options)
        scaledBitmap = Bitmap.createScaledBitmap(scaledBitmap, width, height, true)

        Log.d("Scaled Bitmap -> Width: ${scaledBitmap.width}, Height: ${scaledBitmap.height}")

        return scaledBitmap
    }

    private fun calculateInSampleSize(options: BitmapFactory.Options, reqWidth: Int, reqHeight: Int): Int {
        val height = options.outHeight
        val width = options.outWidth
        var inSampleSize = 1

        if (height > reqHeight || width > reqWidth) {
            val heightRatio = Math.round(height.toFloat() / reqHeight.toFloat())
            val widthRatio = Math.round(width.toFloat() / reqWidth.toFloat())
            inSampleSize = if (heightRatio < widthRatio) heightRatio else widthRatio
        }
        val totalPixels = (width * height).toFloat()
        val totalReqPixelsCap = (reqWidth * reqHeight * 2).toFloat()
        while (totalPixels / (inSampleSize * inSampleSize) > totalReqPixelsCap) {
            inSampleSize++
        }

        return inSampleSize
    }

    companion object : Logger(TAG = "BBC_BitmapPlayGround") {
        init {
            System.loadLibrary("imageprocessor-lib")
        }
    }
}
