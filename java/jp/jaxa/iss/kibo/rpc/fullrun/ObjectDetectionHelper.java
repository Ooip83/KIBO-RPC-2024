package jp.jaxa.iss.kibo.rpc.fullrun;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.core.BaseOptions;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.util.List;

public class ObjectDetectionHelper {
    private static final String TAG = ObjectDetectionHelper.class.getSimpleName();
    private ObjectDetector objectDetector;
    private ImageProcessor imageProcessor;

    public ObjectDetectionHelper(Context context) {
        BaseOptions baseOptions = BaseOptions.builder().setNumThreads(4).build();
        ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions.builder()
                .setScoreThreshold(0.6f)
                .setMaxResults(10)
                .setBaseOptions(baseOptions)
                .build();

        try {
            objectDetector = ObjectDetector.createFromFileAndOptions(context, "ssd_mobilenet_v2_fpnlite_320_metadata_2ndV.tflite", options);
        } catch (IOException e) {
            e.printStackTrace();
        }

        imageProcessor = new ImageProcessor.Builder().build();
    }

    public List<Detection> detectObjects(Mat image) {
        // Convert OpenCV Mat to Bitmap
        Bitmap bitmap = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(image, bitmap);

        // Convert Bitmap to TensorImage
        TensorImage tensorImage = imageProcessor.process(TensorImage.fromBitmap(bitmap));

        // Perform object detection
        return objectDetector.detect(tensorImage);
    }

    public Bitmap drawBoundingBoxes(Mat image, List<Detection> detections) {
        // Convert OpenCV Mat to Bitmap
        Bitmap bitmap = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(image, bitmap);

        // Create a canvas to draw on the bitmap
        Canvas canvas = new Canvas(bitmap);
        Paint boxPaint = new Paint();
        boxPaint.setColor(Color.RED);
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(8);

        Paint textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(50);
        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setAntiAlias(true);
        textPaint.setShadowLayer(10f, 0f, 0f, Color.BLACK);  // Optional: Adds shadow for better readability

        // Draw bounding boxes and labels
        for (Detection detection : detections) {
            // Get bounding box
            RectF box = detection.getBoundingBox();

            // Draw the bounding box
            canvas.drawRect(box, boxPaint);

            // Get label and score
            String label = detection.getCategories().get(0).getLabel();
            float score = detection.getCategories().get(0).getScore();
            String text = String.format("%s: %.4f", label, score);

            // Draw the text (item name and score) above the bounding box
            float textWidth = textPaint.measureText(text);
            canvas.drawText(text, box.left, box.top - 10, textPaint); // Adjust the position as needed
        }

        return bitmap;
    }


    public void logDetectionResults(List<Detection> detections) {
        for (Detection detection : detections) {
            RectF box = detection.getBoundingBox();
            Log.i(TAG, String.format("Detected: %s [%s] at [%s, %s, %s, %s]",
                    detection.getCategories().get(0).getLabel(),
                    detection.getCategories().get(0).getScore(),
                    box.left, box.top, box.right, box.bottom));
        }
    }

    public void close() {
        if (objectDetector != null && !objectDetector.isClosed()) {
            objectDetector.close();
        }
    }
}
