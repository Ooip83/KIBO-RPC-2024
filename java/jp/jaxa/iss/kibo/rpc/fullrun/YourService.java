package jp.jaxa.iss.kibo.rpc.fullrun;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;



public class YourService extends KiboRpcService {
    private static final String TAG = YourService.class.getSimpleName();

    @Override
    protected void runPlan1() {
        // Initialize the ObjectDetectionHelper
        ObjectDetectionHelper objectDetectionHelper = new ObjectDetectionHelper(getApplicationContext());

        // Load AR IDs to be found
        int[] ARIds = new int[5];           // Initialization (creates an array with 5 elements)

        // Initialize parameters
        double markerLength = 0.05; // 5 cm
        final int LOOP_MAX = 5;
        int TargetPlane = 0;

        // AR Ids from programming manuals
        ARIds[0] = 100;
        ARIds[1] = 101;
        ARIds[2] = 102;
        ARIds[3] = 103;
        ARIds[4] = 104;

        // Load centroid of each Areas
        double[][] areaCen = new double[4][6];
        areaCen[0][0] = 10.95; // Area 1: X
        areaCen[0][1] = -9.9; // Area 1: Y (offset to wall added) -- 0.68m away from wall
        areaCen[0][2] = 5.195; // Area 1: Z
        areaCen[0][3] = -90; // Area 1: RZ (yaw)
        areaCen[0][4] = 0; // Area 1: RY (pitch)
        areaCen[0][5] = 0; // Area 1: RX (roll)
        areaCen[1][0] = 10.925; // Area 2: X
        areaCen[1][1] = -8.875; // Area 2: Y
        areaCen[1][2] = 4.46203; // Area 2: Z (offset to wall added) -- 0.7m away from wall
        areaCen[1][3] = 90; // Area 2: RZ (yaw)
        areaCen[1][4] = 90; // Area 2: RY (pitch)
        areaCen[1][5] = 0; // Area 2: RX (roll)
        areaCen[2][0] = 10.925; // Area 3: X
        areaCen[2][1] = -7.925; // Area 3: Y
        areaCen[2][2] = 4.4; // Area 3: Z (offset to wall added) -- change to 4.4
        areaCen[2][3] =  90; // Area 3: RZ (yaw)
        areaCen[2][4] =  90; // Area 3: RY (pitch)
        areaCen[2][5] =  0; // Area 3: RX (roll)
        areaCen[3][0] = 10.56; // Area 4: X (offset to wall added) -- 0.7m away from wall
        areaCen[3][1] = -6.8525; // Area 4: Y
        areaCen[3][2] = 4.945; // Area 4: Z
        areaCen[3][3] = 180; // Area 4: RZ (yaw)
        areaCen[3][4] = 0; // Area 4: RY (pitch)
        areaCen[3][5] = 0; // Area 4: RX (roll)

        // Create empty array to store item type in each Area
        String[] items = new String[5];

        // Create array to save robot kinematic at each Search Plane
        List<Kinematics> posLostItems = new ArrayList<>();


        // The mission starts.
        api.startMission();

        // Get camera matrix and camera coefficient for lens correction
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        MatOfDouble cameraCoefficients = new MatOfDouble(api.getNavCamIntrinsics()[1]);

        // Initialize ArUco Marker Detection
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        for (int i = 0; i < 4; i++) {
            // transition from one Area to another Area
            switch (i){
                case 1:
                    Point pointtrans1 = new Point(10.56, -9.5, 4.62);
                    Quaternion quaterniontrans1 = eulerToQuaternion(90, 0, 0);
                    Log.i("TRANSITION", "Moving to Tunnel 1a");
                    api.moveTo(pointtrans1, quaterniontrans1, false);
                    break;
                case 2:
                    Point pointtrans2 = new Point(11.15, -8.5, 4.62);
                    Quaternion quaterniontrans2 = eulerToQuaternion(90, 0, 0);
                    Log.i("TRANSITION", "Moving to Tunnel 2a");
                    api.moveTo(pointtrans2, quaterniontrans2, false);
                    break;
                case 3:
                    Point pointtrans3 = new Point(10.56, -7.4, 4.62);
                    Quaternion quaterniontrans3 = eulerToQuaternion(90, 0, 0);
                    Log.i("TRANSITION", "Moving to Tunnel 3a");
                    api.moveTo(pointtrans3, quaterniontrans3, false);
                    break;
                default:
                    Log.i("TRANSITION", "No transition needed, proceed to direct motion");

            }

            // Move to centroid
            Point point = new Point(areaCen[i][0], areaCen[i][1], areaCen[i][2]); // Use NASA's Point class
            //Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
            Quaternion quaternion = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
            Log.i("MOVING", "Moving to Area " + String.valueOf(i+1));
            // Moving with time out (to avoid infinite loop)
            Result result;


            // first try
            result = api.moveTo(point, quaternion, true);
            // check result
            int loopCounter = 0;
            while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
                // retry
                result = api.moveTo(point, quaternion, true);
                ++loopCounter;
            }


            boolean itemIdentified = false;
            int counter = 1;
            int backwards = 1;
            int forwards = 1;
            boolean moveaway = false;
            boolean movecloser = false;
            while (!itemIdentified && counter < 6) {
                if (i == 2){
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                Log.i("APPROXIMATING QUADRILATERAL", "Attempting for the "+ counter + "th times.");
                // Take photo and perform undistortion due to lens effect
                // Turn on the front flashlight
                api.flashlightControlFront(0.05f);
                // You might need some sleep…
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                Mat image = api.getMatNavCam(); // CV_8UC1 (grayscale)
                api.flashlightControlFront(0.0f);
                String rawImageName = "Raw_Image_" + String.valueOf(i+1) + ".jpg";
                api.saveMatImage(image, rawImageName); // Save raw image
                // Perform undistortion
                Mat undistortImg = new Mat();
                Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);
                String undistortImageName = "Undistort_Image_" + String.valueOf(i+1) + ".jpg";
                api.saveMatImage(undistortImg, undistortImageName); // Save undistort image

                // Detect ArUco Marker
                List<Mat> corners = new ArrayList<>();
                Mat markerIds = new Mat();
                Aruco.detectMarkers(undistortImg, dictionary, corners, markerIds);

                // Check if ArUco marker exist
                if (!markerIds.empty()) {
                    // Get the corresponding marker ID
                    int markerId = (int) markerIds.get(0, 0)[0];
                    Log.i("ArUco Marker", "Latest Marker ID: " + markerId);
                    // Check if ArUco marker ID matches with Area
                    if (markerId == ARIds[i + 1]) {
                        Log.i("ArUco Marker", "Marker ID matched with Area " + String.valueOf(i+1));
                        // Get corners
                        MatOfPoint2f markerCorners = new MatOfPoint2f(
                                new org.opencv.core.Point(corners.get(0).get(0, 0)),
                                new org.opencv.core.Point(corners.get(0).get(0, 1)),
                                new org.opencv.core.Point(corners.get(0).get(0, 2)),
                                new org.opencv.core.Point(corners.get(0).get(0, 3))
                        );
                        List<MatOfPoint2f> markerCornersList = Collections.singletonList(markerCorners);

                        // call processArucoMarkerPose to give distances (pose estimation)
                        double[] distances = processArucoMarkerPose(markerCornersList, markerLength, cameraMatrix, cameraCoefficients, undistortImg, i, api);
                        if (distances.length == 3) { // if = 0, then pose estimation not successful
                            double distanceX = distances[0];
                            double distanceY = distances[1];
                            double distanceZ = distances[2];

                            // Calculate distance error
                            // Initialize
                            double errorX = 0;
                            double errorY = 0;
                            double errorZ = 0;

                            switch (i){ // Sort of like transformation matrix when rotation occurs
                                case 0: // XZ plane
                                    errorX = distanceX;
                                    errorY = 0;
                                    errorZ = distanceY; // Z in Astrobee
                                    break;
                                case 1 : // XY plane
                                case 2: // XY plane
                                    errorX = -distanceX;
                                    errorY = distanceY;
                                    errorZ = 0;
                                    break;
                                case 3: // YZ plane
                                    errorX = 0;
                                    errorY = -distanceX;
                                    errorZ = distanceY;
                                    break;
                                default:

                            }

                            // Move to a corrected point.
                            Kinematics pos1 = api.getRobotKinematics();
                            Point corpoint = new Point(pos1.getPosition().getX() + errorX, pos1.getPosition().getY() + errorY, pos1.getPosition().getZ() + errorZ); //no constants added for paper center to ArUco marker offset
                            Quaternion corquaternion = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]); // no change
                            Log.i("MOVING", "Moving to Corrected Point");
                            api.moveTo(corpoint, corquaternion, false);


                            // Take NEW PHOTO and perform undistortion due to lens effect
                            // Turn on the front flashlight
                            api.flashlightControlFront(0.05f);
                            // You might need some sleep…
                            try {
                                Thread.sleep(1000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            Mat imageCorrected = api.getMatNavCam(); // CV_8UC1 (grayscale)
                            // Turn off the front flashlight
                            api.flashlightControlFront(0.0f);
                            String rawCorrectedImageName = "Raw_Corrected_Image_" + String.valueOf(i+1) + ".jpg";
                            api.saveMatImage(imageCorrected, rawCorrectedImageName); // Save raw image
                            // Perform undistortion
                            Mat undistortCorImg = new Mat();
                            Calib3d.undistort(imageCorrected, undistortCorImg, cameraMatrix, cameraCoefficients);
                            String undistortCorImageName = "Undistort_Corrected_Image_" + String.valueOf(i+1) + ".jpg";
                            api.saveMatImage(undistortCorImg, undistortCorImageName); // Save corrected undistort image

                            // Get new ArUco marker corners
                            Aruco.detectMarkers(undistortCorImg, dictionary, corners, markerIds);
                            if (!markerIds.empty()) {
                                MatOfPoint2f markerCornersnew = new MatOfPoint2f(
                                        new org.opencv.core.Point(corners.get(0).get(0, 0)),
                                        new org.opencv.core.Point(corners.get(0).get(0, 1)),
                                        new org.opencv.core.Point(corners.get(0).get(0, 2)),
                                        new org.opencv.core.Point(corners.get(0).get(0, 3))
                                );

                                // Proceed with perspective transformation
                                Mat transformedImage = performPerspectiveTransformation(undistortCorImg, markerCornersnew, i, api);

                                // Check if transformation was successful (quadrilateral present), then run inference
                                if(transformedImage.empty()){ // quadrilateral failed to approximated
                                    if (counter > 2){ // ready to move away from wall as poor illumination suspected
                                        moveaway = true;
                                        Log.i("FINDING PAPER", "Attempt to move away from wall");

                                    }
                                    Log.i("FINDING PAPER", "Attempt to approach again");

                                }else { // Proceed with the transformed image

                                    // But first check if ArUco marker still present or not
                                    // Confirm ArUco marker still recognizable after transformation
                                    Mat markerIdsafter = new Mat();
                                    Aruco.detectMarkers(transformedImage, dictionary, corners, markerIdsafter);
                                    if (!markerIdsafter.empty()) {
                                        int markerIdafter = (int) markerIdsafter.get(0, 0)[0];
                                        if (markerIdafter == ARIds[i + 1]) {
                                            Log.i("ArUco Marker", "Still recognizable");
                                            // Detect objects in the warped image
                                            List<Detection> detections = objectDetectionHelper.detectObjects(transformedImage);

                                            // Log detection results
                                            objectDetectionHelper.logDetectionResults(detections);

                                            // Draw bounding boxes on the image
                                            Bitmap resultBitmap = objectDetectionHelper.drawBoundingBoxes(transformedImage, detections);

                                            // Convert Bitmap back to Mat and save the result image
                                            Mat resultMat = new Mat();
                                            Utils.bitmapToMat(resultBitmap, resultMat);
                                            String DetectedName = "Detected_Objects_Area_" + String.valueOf(i + 1) + ".jpg";
                                            api.saveMatImage(resultMat, DetectedName);

                                            // Process detections and set area information
                                            int detectedItemCount = detections.size();
                                            if (detectedItemCount > 0) {
                                                String detectedItemName = detections.get(0).getCategories().get(0).getLabel();
                                                api.setAreaInfo(i + 1, detectedItemName, detectedItemCount);
                                                itemIdentified = true; // exit loop of approaching ArUco Marker
                                                items[i] = detectedItemName;
                                                Log.i("Area " + String.valueOf(i + 1) + " Item:", detectedItemName);

                                                // Save current position and orientation after lost items is identified
                                                Kinematics k = api.getRobotKinematics();
                                                posLostItems.add(k);
                                                Log.i("LOGGING DATA", "Logged current kinematics for Area " + String.valueOf(i + 1));


                                            } else {
                                                Log.i("RUNNING INFERENCE STATUS: ", "FAILED, ATTEMPT TO MOVE CLOSER");
                                                //api.setAreaInfo(i + 1, "unknown", 0);
                                                movecloser = true;

                                            }
                                        } else {
                                            Log.i("ArUco Marker", "Not recognizable after transformation, retry");
                                            movecloser = true;
                                        }
                                    }else{
                                        Log.i("ArUco Marker", "Not detected after transformation, retry");
                                    }

                                }



                            } else {
                                Log.i("Wrong Movement", "Logical error, went to wrong direction, pls review code, attempt moving away");
                                moveaway = true;
                            }


                        } else {
                            Log.i("ArUco Marker", "Mostly impossible, maybe bad image quality");
                        }
                    } else {
                        Log.i("Wrong Area", "Logical error, went to wrong area, pls review code, attempt moving away");
                        moveaway = true;

                    }


                } else {
                    Log.i("ArUco Marker", "No ArUco markers detected.");
                    // begin search pattern
                    // start with moving away from wall
                    Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving away from wall");
                    moveaway = true;
                }
                if (moveaway == true) {
                    Kinematics posmovaway = api.getRobotKinematics();
                    switch (i) {
                        case 0: // XZ Plane
                            Point pointsearch0 = new Point(posmovaway.getPosition().getX(), posmovaway.getPosition().getY() + 0.1 * backwards, posmovaway.getPosition().getZ());
                            Quaternion quaternionsearch0 = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving away from wall");
                            api.moveTo(pointsearch0, quaternionsearch0, false);
                            moveaway = false;
                            break;
                        case 1: // XY Plane
                        case 2: // XY Plane
                            Point pointsearch2 = new Point(posmovaway.getPosition().getX(), posmovaway.getPosition().getY(), posmovaway.getPosition().getZ() + 0.1 * backwards);
                            Quaternion quaternionsearch2 = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving away from wall");
                            api.moveTo(pointsearch2, quaternionsearch2, false);
                            moveaway = false;
                            break;
                        case 3: // YZ Plane
                            Point pointsearch3 = new Point(posmovaway.getPosition().getX() + 0.1 * backwards, posmovaway.getPosition().getY(), posmovaway.getPosition().getZ());
                            Quaternion quaternionsearch3 = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving away from wall");
                            api.moveTo(pointsearch3, quaternionsearch3, false);
                            moveaway = false;
                            break;
                        default:
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Impossible to get it here");
                            moveaway = false;

                    }

                }
                if (movecloser == true) {
                    Kinematics posmovcloser = api.getRobotKinematics();
                    switch (i) {
                        case 0: // XZ Plane
                            Point pointsearch0 = new Point(posmovcloser.getPosition().getX(), posmovcloser.getPosition().getY() + 0.1 * forwards, posmovcloser.getPosition().getZ());
                            Quaternion quaternionsearch0 = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving closer to wall");
                            api.moveTo(pointsearch0, quaternionsearch0, false);
                            movecloser = false;
                            break;
                        case 1: // XY Plane
                        case 2: // XY Plane
                            Point pointsearch2 = new Point(posmovcloser.getPosition().getX(), posmovcloser.getPosition().getY(), posmovcloser.getPosition().getZ() + 0.1 * forwards);
                            Quaternion quaternionsearch2 = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving closer to wall");
                            api.moveTo(pointsearch2, quaternionsearch2, false);
                            movecloser = false;
                            break;
                        case 3: // YZ Plane
                            Point pointsearch3 = new Point(posmovcloser.getPosition().getX() + 0.1 * forwards, posmovcloser.getPosition().getY(), posmovcloser.getPosition().getZ());
                            Quaternion quaternionsearch3 = eulerToQuaternion(areaCen[i][3], areaCen[i][4], areaCen[i][5]);
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Moving closer to wall");
                            api.moveTo(pointsearch3, quaternionsearch3, false);
                            movecloser = false;
                            break;
                        default:
                            Log.i("ATTEMPT TO FIND ARUCO MARKER", "Impossible to get it here");
                            movecloser = false;

                    }

                }



                counter ++;
            }



        }

        // Move to astronaut spot
        Point pointAstro = new Point(11.143, -6.76, 4.9654); // need within 0.3m
        Quaternion quaternionAstro = new Quaternion(0f, 0f, 0.707f, 0.707f);
        Log.i("MOVING", "Moving to astronaut");
        api.moveTo(pointAstro, quaternionAstro, true);
        // Report rounding completion after went to all 4 Areas
        api.reportRoundingCompletion();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Try recognizing what the item the Astronaut is holding
        // Point pointRe = new Point(11.143, -7.4, 4.9654);
        // Quaternion quaternionRe = new Quaternion(0f, 0f, 0.707f, 0.707f);
        // Log.i("MOVING", "Moving away from astronaut");
        // first try
        // Result resultgo;
        // resultgo = api.moveTo(pointRe, quaternionRe, true);
        // check result
        // int loopCounter = 0;
        // while(!resultgo.hasSucceeded() && loopCounter < LOOP_MAX){
            // retry
            // resultgo = api.moveTo(pointRe, quaternionRe, true);
            // ++loopCounter;
        // }

        // Take image to recognize target item
        // Turn on the front flashlight
        api.flashlightControlFront(0.05f);
        // You might need some sleep…
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mat Tarimage = api.getMatNavCam();
        api.flashlightControlFront(0.0f);
        String TarImageName = "Tar_Image.jpg";
        api.saveMatImage(Tarimage, TarImageName); // Save target image
        // Perform undistortion
        Mat undistortTarImg = new Mat();
        Calib3d.undistort(Tarimage, undistortTarImg, cameraMatrix, cameraCoefficients);
        String undistortTarImageName = "Undistort_Target_Image.jpg";
        api.saveMatImage(undistortTarImg, undistortTarImageName); // Save undistort target image

        Mat markerIdsRe = new Mat();
        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(undistortTarImg, dictionary, corners, markerIdsRe);
        int markerIdRe = (int) markerIdsRe.get(0, 0)[0];
        if (markerIdRe == 100){
            Log.i("ArUco Marker", "Target marker recognizable");
                MatOfPoint2f markerCornersnew = new MatOfPoint2f(
                        new org.opencv.core.Point(corners.get(0).get(0, 0)),
                        new org.opencv.core.Point(corners.get(0).get(0, 1)),
                        new org.opencv.core.Point(corners.get(0).get(0, 2)),
                        new org.opencv.core.Point(corners.get(0).get(0, 3))
                );

                // Proceed with perspective transformation
                Mat transformedImageRe = performPerspectiveTransformation(undistortTarImg, markerCornersnew, 5, api);

                // Detect objects in the warped image
                List<Detection> detections = objectDetectionHelper.detectObjects(transformedImageRe);

                // Log detection results
                objectDetectionHelper.logDetectionResults(detections);

                // Draw bounding boxes on the image
                Bitmap resultBitmap = objectDetectionHelper.drawBoundingBoxes(transformedImageRe, detections);

                // Convert Bitmap back to Mat and save the result image
                Mat resultMat = new Mat();
                Utils.bitmapToMat(resultBitmap, resultMat);
                String DetectedName = "Target_Objects.jpg";
                api.saveMatImage(resultMat, DetectedName);

                // Process detections and set area information

                int detectedItemCount = detections.size();
                if (detectedItemCount > 0) {
                    items[4] = detections.get(0).getCategories().get(0).getLabel();
                    Log.i("Target Item: ", items[4]);
                } else {
                    Log.i("RUNNING INFERENCE STATUS: ", "FAILED");
                }
            }else{
                Log.i("ArUco Marker", "Pls experiment again");

            }

        // Notify item recognise
        api.notifyRecognitionItem();

        // Check target item belongs to which Area

        for (int c = 0; c < 4; c++){
            if (items[c].equals(items[4])){
                Log.i("Target Item", "Target item is: " + items[c]);
                TargetPlane = c;
                Log.i("Target Item", "Target item is in Area " + String.valueOf(TargetPlane+1));
            }
        }

        // Add logic to move to target item area (save for later)
        Kinematics TarKinematics = posLostItems.get(TargetPlane);
        Point pointTar = new Point(TarKinematics.getPosition().getX(), TarKinematics.getPosition().getY(), TarKinematics.getPosition().getZ());
        Quaternion quaternionTar = new Quaternion(TarKinematics.getOrientation().getX(), TarKinematics.getOrientation().getY(), TarKinematics.getOrientation().getZ(), TarKinematics.getOrientation().getW());

        // Transition (move across different Areas)
        switch (TargetPlane){
            case 0: // cross 3 KOZ
                // KOZ 3
                Point pointtrans03 = new Point(10.56, -7.4, 4.62);
                Quaternion quaterniontrans03 = eulerToQuaternion(-90, 0, 0);
                Log.i("TRANSITION", "Moving to Tunnel 3a");
                api.moveTo(pointtrans03, quaterniontrans03, true);

                // KOZ 2
                Point pointtrans02 = new Point(11.15, -8.5, 4.62);
                Quaternion quaterniontrans02 = eulerToQuaternion(-90, 0, 0);
                Log.i("TRANSITION", "Moving to Tunnel 2a");
                api.moveTo(pointtrans02, quaterniontrans02, true);

                // KOZ 1
                Point pointtrans01 = new Point(10.56, -9.5, 4.62);
                Quaternion quaterniontrans01 = eulerToQuaternion(-90, 0, 0);
                Log.i("TRANSITION", "Moving to Tunnel 1a");
                api.moveTo(pointtrans01, quaterniontrans01, true);
                break;
            case 1: // cross 2 KOZ
                // KOZ 3
                Point pointtrans13 = new Point(10.56, -7.4, 4.62);
                Quaternion quaterniontrans13 = eulerToQuaternion(-90, 0, 0);
                Log.i("TRANSITION", "Moving to Tunnel 3a");
                api.moveTo(pointtrans13, quaterniontrans13, true);

                // KOZ 2
                Point pointtrans12 = new Point(11.15, -8.5, 4.62);
                Quaternion quaterniontrans12 = eulerToQuaternion(-90, 0, 0);
                Log.i("TRANSITION", "Moving to Tunnel 2a");
                api.moveTo(pointtrans12, quaterniontrans12, true);
                break;
            case 2: // cross 1 KOZ
                // KOZ 3
                Point pointtrans23 = new Point(10.56, -7.4, 4.62);
                Quaternion quaterniontrans23 = eulerToQuaternion(-90, 0, 0);
                Log.i("TRANSITION", "Moving to Tunnel 3a");
                api.moveTo(pointtrans23, quaterniontrans23, true);
                break;
            default:
                Log.i("TRANSITION", "No transition needed, proceed to direct motion");
        }

        // Moving with time out (to avoid infinite loop)
        Result result;
        // first try
        result = api.moveTo(pointTar, quaternionTar, true);
        // check result
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            // retry
            result = api.moveTo(pointTar, quaternionTar, true);
            ++loopCounter;
        }

        api.takeTargetItemSnapshot();
        objectDetectionHelper.close();
    }

    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree) {
        double yaw = Math.toRadians(yaw_degree); //radian = degree*PI/180
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }

    public double[] processArucoMarkerPose (
            List<MatOfPoint2f> markerCorners,
            double markerLength,
            Mat cameraMatrix,
            Mat cameraCoefficients,
            Mat undistortImg,
            int index,
            KiboRpcApi api) {
        try {
            // Pose estimation of the ArUco marker
            Mat rvecs = new Mat();  // Rotation vectors
            Mat tvecs = new Mat();  // Translation vectors

            // Estimate pose of the single ArUco marker
            Aruco.estimatePoseSingleMarkers(Collections.unmodifiableList(markerCorners), (float) markerLength, cameraMatrix, cameraCoefficients, rvecs, tvecs);

            // Get the translation vector (x, y, z)
            double[] translation = tvecs.get(0, 0);
            double distanceX = translation[0];
            double distanceY = translation[1];
            double distanceZ = translation[2];

            // Log the distance (translation vector) from camera to ArUco marker
            Log.i("ArUco Marker", "Translation Vector (x, y, z): " + distanceX + ", " + distanceY + ", " + distanceZ);

            // Annotate the distance on the image
            String distanceText = String.format("Distance to Marker (m): [X: %.3f, Y: %.3f, Z: %.3f]", distanceX, distanceY, distanceZ);
            Mat annotatedImg = undistortImg.clone(); // Clone the image to avoid modifying the original
            Imgproc.putText(annotatedImg, distanceText, new org.opencv.core.Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), 2);

            // Project the 3D point of the ArUco marker center to 2D
            MatOfPoint2f projectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(new MatOfPoint3f(new org.opencv.core.Point3(0, 0, 0)), rvecs, tvecs, cameraMatrix, (MatOfDouble) cameraCoefficients, projectedPoints);

            // Get the image center (assumed to be the camera center)
            int imageCenterX = undistortImg.cols() / 2;
            int imageCenterY = undistortImg.rows() / 2;
            org.opencv.core.Point imageCenter = new org.opencv.core.Point(imageCenterX, imageCenterY);

            // Get the projected 2D point of the ArUco marker center
            org.opencv.core.Point markerCenter2D = projectedPoints.toArray()[0];

            // Draw a line from the camera center to the projected ArUco marker center (black)
            Imgproc.line(annotatedImg, imageCenter, markerCenter2D, new Scalar(0, 0, 0), 2);

            // Draw a red line for the X distance (horizontal line)
            org.opencv.core.Point xEndPoint = new org.opencv.core.Point(markerCenter2D.x, imageCenter.y); // Horizontal end point
            Imgproc.line(annotatedImg, imageCenter, xEndPoint, new Scalar(0, 255, 0), 2); // Red line

            // Draw a blue line for the Y distance (vertical line)
            org.opencv.core.Point yEndPoint = new org.opencv.core.Point(imageCenter.x, markerCenter2D.y); // Vertical end point
            Imgproc.line(annotatedImg, imageCenter, yEndPoint, new Scalar(255, 0, 0), 2); // Blue line

            // Optionally, annotate the X and Y distances on the image
            String xLabel = "X = " + String.format("%.3f", markerCenter2D.x - imageCenter.x) + " px";
            String yLabel = "Y = " + String.format("%.3f", markerCenter2D.y - imageCenter.y) + " px";

            // Display the X distance text near the end of the red line
            Imgproc.putText(annotatedImg, xLabel, new org.opencv.core.Point(xEndPoint.x + 5, xEndPoint.y - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);

            // Display the Y distance text near the end of the blue line
            Imgproc.putText(annotatedImg, yLabel, new org.opencv.core.Point(yEndPoint.x + 5, yEndPoint.y - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 1);

            // Save the annotated image with the line
            String ArucoAnnotated = "Vectors_Annotated_" + String.valueOf(index + 1) + ".jpg";
            api.saveMatImage(annotatedImg, ArucoAnnotated);

            // Return the distances
            return new double[] { distanceX, distanceY, distanceZ };

        } catch (Exception e) {
            Log.e("ArucoPose", "Error processing ArUco marker pose: " + e.getMessage());
            return new double[0]; // Return an empty array if there's an error
        }
    }

    public Mat performPerspectiveTransformation(
            Mat undistortCorImg,
            MatOfPoint2f markerCornersnew,
            int index,
            KiboRpcApi api // Replace with actual class
            ) {

        Mat warped = new Mat();
        try {
            // Proceed with perspective transformation
            int cappedThres = 0;
            Mat gray = undistortCorImg;
            if (index == 5){
                cappedThres = 190;
            }
            else{
                cappedThres = 220;
            }
            int minThres = 250;
            MatOfPoint2f approxPoly = new MatOfPoint2f();  // To store the approximated quadrilateral
            boolean quadrilateralFound = false;

            while (minThres > cappedThres) {
                Mat binary = new Mat();
                Imgproc.threshold(gray, binary, minThres, 255, Imgproc.THRESH_BINARY);

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Assuming the largest contour is the white paper
                if (!contours.isEmpty()) {
                    MatOfPoint largestContour = contours.stream()
                            .max((c1, c2) -> Double.compare(Imgproc.contourArea(c1), Imgproc.contourArea(c2)))
                            .orElse(null);

                    if (largestContour != null) {
                        MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());
                        double epsilon = 0.02 * Imgproc.arcLength(contour2f, true); // Adjust approximation accuracy as required
                        Imgproc.approxPolyDP(contour2f, approxPoly, epsilon, true);

                        // Ensure it's a quadrilateral (exactly 4 points)
                        if (approxPoly.rows() == 4) {
                            quadrilateralFound = true;
                            Log.i("FINDING PAPER", "Successful");
                            break;
                        }
                    }
                }

                // Reduce threshold if no quadrilateral is found
                minThres -= 10;
                Log.i("FINDING PAPER", "Reducing minThres value: " + minThres);
            }

            if (!quadrilateralFound) {
                // No quadrilateral detected, skip object detection and end the mission
                Log.i("FINDING PAPER", "Can't approximate quadrilateral!");
                // api.setAreaInfo(1, "unknown_outofframe", 0);
                return warped; // Return empty Mat if no quadrilateral is found
            }

            // Quadrilateral found, proceed with perspective transformation
            CanvasAligner canvasAligner = new CanvasAligner();
            org.opencv.core.Point[] paperCorners = approxPoly.toArray();  // Assume these are the corners of the detected quadrilateral

            // Use detected ArUco marker corners
            org.opencv.core.Point[] markerCornersArray = {
                    markerCornersnew.toArray()[0],
                    markerCornersnew.toArray()[1],
                    markerCornersnew.toArray()[2],
                    markerCornersnew.toArray()[3]
            };

            // Perform perspective transformation using CanvasAligner
            warped = canvasAligner.alignCanvas(undistortCorImg, paperCorners, markerCornersArray); // Perspective transform

            // Save cropped image after perspective transform
            String outputTransformName = "Output_Transformed_Image_" + (index + 1) + ".jpg";
            api.saveMatImage(warped, outputTransformName);

        } catch (Exception e) {
            Log.e("PerspectiveTransform", "Error performing perspective transformation: " + e.getMessage());
        }

        return warped; // Return the transformed image
    }




}
