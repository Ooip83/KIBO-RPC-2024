package jp.jaxa.iss.kibo.rpc.fullrun;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.Comparator;

public class CanvasAligner {

    // Calculate the centroid of the paper corners
    private Point calculateCentroid(Point[] paperCorners) {
        double sumX = 0, sumY = 0;
        for (Point point : paperCorners) {
            sumX += point.x;
            sumY += point.y;
        }
        return new Point(sumX / paperCorners.length, sumY / paperCorners.length);
    }

    // Sort corners based on their angle from the centroid
    private Point[] sortCornersByAngle(Point[] paperCorners, final Point centroid) {
        Comparator<Point> angleComparator = (p1, p2) -> {
            double angle1 = angleFromCentroid(p1, centroid);
            double angle2 = angleFromCentroid(p2, centroid);
            return Double.compare(angle1, angle2);
        };
        return Arrays.stream(paperCorners).sorted(angleComparator).toArray(Point[]::new);
    }

    // Calculate angle of a point relative to the centroid
    private double angleFromCentroid(Point p, Point centroid) {
        return Math.atan2(p.y - centroid.y, p.x - centroid.x);
    }

    // Compute the Euclidean distance between two points
    private double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    // Perform perspective transform
    public Mat alignCanvas(Mat inputImage, Point[] paperCorners, Point[] markerCorners) {
        // Step 1: Calculate the centroid
        Point centroid = calculateCentroid(paperCorners);

        // Step 2: Sort the corners based on their angle from the centroid
        Point[] sortedCorners = sortCornersByAngle(paperCorners, centroid);

        // Step 3: Find the top-right marker corner (assuming it's at index 1)
        Point topRightMarker = markerCorners[1];

        // Step 4: Find the closest corner of the paper to the marker's top-right corner
        Point closestCorner = null;
        double minDistance = Double.MAX_VALUE;
        int closestCornerIndex = -1;

        for (int i = 0; i < sortedCorners.length; i++) {
            double dist = distance(topRightMarker, sortedCorners[i]);
            if (dist < minDistance) {
                minDistance = dist;
                closestCorner = sortedCorners[i];
                closestCornerIndex = i;
            }
        }

        // Step 5: Resort the corners based on the closest corner index
        int[] resortOrder = {3, 0, 1, 2}; // Default order
        if (closestCornerIndex == 0) {
            resortOrder = new int[]{3, 0, 1, 2};
        } else if (closestCornerIndex == 2) {
            resortOrder = new int[]{1, 2, 3, 0};
        } else if (closestCornerIndex == 3) {
            resortOrder = new int[]{2, 3, 0, 1};
        } else {
            resortOrder = new int[]{0, 1, 2, 3};
        }

        // Step 6: Resort corners
        Point[] resortedCorners = new Point[4];
        for (int i = 0; i < 4; i++) {
            resortedCorners[i] = sortedCorners[resortOrder[i]];
        }

        // Step 7: Define the destination points for the perspective transform
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(0, 0),          // Top-left corner
                new Point(1280, 0),       // Top-right corner
                new Point(1280, 960),     // Bottom-right corner
                new Point(0, 960)         // Bottom-left corner
        );

        // Convert the points to float
        MatOfPoint2f srcPoints = new MatOfPoint2f(resortedCorners);

        // Step 8: Calculate the perspective transform matrix
        Mat perspectiveMatrix = Imgproc.getPerspectiveTransform(srcPoints, dstPoints);

        // Step 9: Warp the input image
        Mat outputImage = new Mat();
        Imgproc.warpPerspective(inputImage, outputImage, perspectiveMatrix, new Size(1280, 960));

        return outputImage;
    }
}
