package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

// Detecting contours of specific colors
public class ContourDetectionPipeline extends OpenCvPipeline {
   private Mat output = new Mat();
   // NOTE: "Scalar" color channels are RGB
   private Scalar low_hsv = new Scalar(0, 75, 75);
   private Scalar high_hsv = new Scalar(10, 255, 255);
   private final Scalar CONTOUR_COLOR = new Scalar(0, 0, 125);
   // Mat variables used for find_contours() method
   private Mat hsv = new Mat();
   private Mat mask = new Mat();
   private Mat hierarchy = new Mat(); // NOT necessary for most tasks
   private MatOfPoint max_contour;
   Point contour_center = new Point();

   @Override
   public Mat processFrame(Mat input) {
      // finds contours
      ArrayList<MatOfPoint> contour_list = find_contours(input, low_hsv, high_hsv);
      input.copyTo(output);
      // finding max contour
      MatOfPoint max_contour = contour_list.get(0);
      double max_contour_area = -1;
      for (MatOfPoint contour : contour_list) {
         if (Imgproc.contourArea(contour) > max_contour_area) {
            max_contour = contour;
            max_contour_area = Imgproc.contourArea(contour);
         }
      }
      // gets center
      Point contour_center = get_contour_center();

      // draws contours
      ArrayList<MatOfPoint> contour = new ArrayList<>();
      contour.add(max_contour);
      Imgproc.drawContours(output, contour, -1, CONTOUR_COLOR, 12);
      Imgproc.circle(output, contour_center, 16, new Scalar(0, 255, 0), -1);

      return output;
   }

   // gets a list of contours
   public ArrayList<MatOfPoint> find_contours(Mat image, Scalar low, Scalar high) {

      ArrayList<MatOfPoint> contours = new ArrayList<>();

      Imgproc.cvtColor(image, hsv, Imgproc.COLOR_RGB2HSV); // converts RGB image to HSV
      Core.inRange(hsv, low, high, mask); // creating mask
      Imgproc.findContours(mask, contours, hierarchy,
              Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

      return contours;
   }

   public Point get_contour_center() {
      // Compute moments
      Moments moments = Imgproc.moments(max_contour);

      // Center X Calculation
      int cx = (int) (moments.get_m10() / moments.get_m00());
      int cy = (int) (moments.get_m01() / moments.get_m00());

      this.contour_center = new Point(cx, cy);
      return contour_center;
   }

}