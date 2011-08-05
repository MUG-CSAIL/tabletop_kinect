package edu.mit.yingyin.tabletop;

import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point2f;

import static com.googlecode.javacv.cpp.opencv_core.CV_32FC1;
import static com.googlecode.javacv.cpp.opencv_core.CV_32FC2;
import static com.googlecode.javacv.cpp.opencv_calib3d.cvFindExtrinsicCameraParams2;
import static com.googlecode.javacv.cpp.opencv_calib3d.cvRodrigues2;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvUndistortPoints;
import static com.googlecode.javacv.cpp.opencv_core.cvGEMM;
import static com.googlecode.javacv.cpp.opencv_core.cvTranspose;

import com.googlecode.javacv.cpp.opencv_core.CvMat;

public class CalibrationExample {
  private static final float[][] INTRINSIC_MATRIX = {
      {(float)5.9421434211923247e+02, 0, (float)3.3930780975300314e+02},
      {0, (float)5.9104053696870778e+02, (float)2.4273913761751615e+02},
      {0, 0, 1}};
  
  private static final float[] DISTORTION_COEFFS = {
      (float)-2.6386489753128833e-01, (float)9.9966832163729757e-01, 
      (float)-7.6275862143610667e-04, (float)5.0350940090814270e-03,
      (float)-1.3053628089976321e+00};
  
  private CvMat rotationMat = CvMat.create(3, 3, CV_32FC1);
  private CvMat translationMat = CvMat.create(3, 1, CV_32FC1);
  private CvMat intrinsicMatrixMat = CvMat.create(3, 3, CV_32FC1);
  private CvMat distortionCoeffsMat = CvMat.create(5, 1, CV_32FC1);
  
  /**
   * Constructs a CalibrationExample from corresponding object points and image
   * points.
   * 
   * @param objectPoints a list of object points in 2D.
   * @param imagePoints a list of image points in 2D.
   */
  public CalibrationExample(List<Point2f> objectPoints, 
      List<Point2f> imagePoints) {
    for (int i = 0; i < 3; i++) 
      for (int j = 0; j < 3; j++) {
        intrinsicMatrixMat.put(i, j, INTRINSIC_MATRIX[i][j]);
      }
    for (int i = 0; i < 5; i++) {
      distortionCoeffsMat.put(i, DISTORTION_COEFFS[i]);
    }
    findExtrinsicCameraParams(objectPoints, imagePoints);
  }
  
  /**
   * Transforms image coordinates to display coordinates P_d.
   * 
   * @param imagePoint 2D point in image coordinates.
   * @return 2D point in display coordinates.
   */
  public Point2f imageToDisplayCoords(Point2f imagePoint) {
    CvMat imageCoords = CvMat.create(1, 1, CV_32FC2);
    CvMat undistorted = CvMat.create(1, 1, CV_32FC2);
    imageCoords.put(0, imagePoint.x);
    imageCoords.put(1, imagePoint.y);
    cvUndistortPoints(imageCoords, undistorted, intrinsicMatrixMat, 
        distortionCoeffsMat, null, null);
    
    // P_cn = [X_c / Z_c, Y_c / Z_c, 1]
    // P_c = Z_c * P_cn
    CvMat normalizedCameraCoords = CvMat.create(3, 1, CV_32FC1);
    normalizedCameraCoords.put(0, undistorted.get(0));
    normalizedCameraCoords.put(1, undistorted.get(1));
    normalizedCameraCoords.put(2, 1);

    // P_c = R * P_d + T
    // P_d = R^-1 * (P_c - T) = Z_c * (R^-1 * P_cn) - R^-1 * T
    CvMat rotationInverse = CvMat.create(3, 3, CV_32FC1);
    cvTranspose(rotationMat, rotationInverse);
    
    // result1 = R^-1 * P_cn
    CvMat result1 = CvMat.create(3, 1, CV_32FC1);
    // result2 = R^-1 * T
    CvMat result2 = CvMat.create(3, 1, CV_32FC1);
    cvGEMM(rotationInverse, normalizedCameraCoords, 1, null, 0, result1, 0);
    cvGEMM(rotationInverse, translationMat, 1, null, 0, result2, 0);
    
    float cameraZ = (float)(result2.get(2) / result1.get(2));

    Point2f converted = new Point2f(
        (float)(result1.get(0) * cameraZ - result2.get(0)),
        (float)(result1.get(1) * cameraZ - result2.get(1)));
    
    imageCoords.release();
    undistorted.release();
    normalizedCameraCoords.release();
    rotationInverse.release();
    result1.release();
    result2.release();

    return converted;
  }
  
  /**
   * Calculates the average error between the converted display coordinates from 
   * the image coordinates and the actual display coordinates.
   * 
   * @param displayCoords list of actaul display coordinates.
   * @param imageCoords list of corresponding image coordinates of the same size
   *     as displayCoords.
   * @return average of squared L2 distance between converted and actual display
   *     coordinates.
   */
  public float imageToDisplayCoordsError(List<Point2f> displayCoords,
      List<Point2f> imageCoords) {
    float error = 0;
    for (int i = 0; i < displayCoords.size(); i++) {
      Point2f converted = imageToDisplayCoords(imageCoords.get(i));
      error += converted.distanceSquared(displayCoords.get(i));
    }
    return error / displayCoords.size();
  }
  
  public void release() {
    rotationMat.release();
    translationMat.release();
    intrinsicMatrixMat.release();
    distortionCoeffsMat.release();
  }
  
  public String toString() {
    StringBuffer sb = new StringBuffer();
    sb.append("Rotation matrix: [");
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        sb.append(rotationMat.get(i, j) + " ");
      }
    sb.append("]\n");
    sb.append("Translation matrixt: [");
    for (int i = 0; i < 3; i++)
      sb.append(translationMat.get(i) + " ");
    sb.append("]");
    return sb.toString();
  }
  
  /**
   * Requires objectPoints.size = imagePoints.size
   * @param objectPoints
   * @param imagePoints
   */
  private void findExtrinsicCameraParams(List<Point2f> objectPoints,
      List<Point2f> imagePoints) {
    CvMat objectPointsMat = CvMat.create(objectPoints.size(), 3, CV_32FC1);
    CvMat imagePointsMat = CvMat.create(imagePoints.size(), 2, CV_32FC1);
    CvMat rodrigues = CvMat.create(3, 1, CV_32FC1);
    
    for (int i = 0; i < objectPoints.size(); i++) {
      objectPointsMat.put(i, 0, objectPoints.get(i).x);
      objectPointsMat.put(i, 1, objectPoints.get(i).y);
      objectPointsMat.put(i, 2, 0);
      
      imagePointsMat.put(i, 0, imagePoints.get(i).x);
      imagePointsMat.put(i, 1, imagePoints.get(i).y);
    }
    
    cvFindExtrinsicCameraParams2(objectPointsMat, imagePointsMat, 
        intrinsicMatrixMat, distortionCoeffsMat, rodrigues, translationMat);
    cvRodrigues2(rodrigues, rotationMat, null);
    objectPointsMat.release();
    imagePointsMat.release();
    rodrigues.release();
  }
}