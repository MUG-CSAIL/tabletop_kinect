package edu.mit.yingyin.tabletop.models;

import static com.googlecode.javacv.cpp.opencv_core.CV_32FC3;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_DIST_L2;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvFitLine;

import com.googlecode.javacv.cpp.opencv_core.CvMat;
import com.googlecode.javacv.cpp.opencv_core.CvRect;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Logger;

import javax.media.j3d.LinearFog;
import javax.vecmath.Point3f;

import org.OpenNI.Point3D;
import org.OpenNI.StatusException;

import edu.mit.yingyin.util.FilteredPoint3fs;
import edu.mit.yingyin.util.Geometry;

/**
 * Handles deictic gestures. Tasks this object performs include computing 
 * the point on the surface this hand is pointing at.
 * @author yingyin
 *
 */
public class DiecticGestureHandler {
  private static final Logger LOGGER = Logger.getLogger(
      DiecticGestureHandler.class.getName());
  
  private OpenNIDevice openni;
  
  /**
   * Number of points to use for filter smoothing.
   */
  private final int FILTER_HISTORY_LENGTH = 2;
  private FilteredPoint3fs filteredFingertip, filteredArmjoint;
  
  /**
   * Must be at least 2.
   */
  private final int NUM_ARM_SUBSAMPLE_POINTS = 10; 
  
  private final int RECT_RADIUS = 5;
  
  
  public DiecticGestureHandler(OpenNIDevice openni) {
    filteredFingertip = new FilteredPoint3fs(FILTER_HISTORY_LENGTH);
    filteredArmjoint = new FilteredPoint3fs(FILTER_HISTORY_LENGTH);
    this.openni = openni;
  }
  
  /**
   * @param forelimbs
   * @return a list of intersections of diectic gestures.
   */
  public List<Point3D> update(List<Forelimb> forelimbs, ProcessPacket packet) {
    InteractionSurface is = InteractionSurface.instance();
    List<Point3D> res = new ArrayList<Point3D>();
    if (is == null)
      return res;
    
    for (Forelimb fl : forelimbs) {
      Point3D p = computeIntersection(fl, is, packet);
      if (p != null) {
        res.add(p);
      }
    }
    return res;
  }
  
  /**
   * 
   * @param fl
   * @param is
   * @return null if there is no fingertip or arm joint in {@code fl}.
   */
  private Point3D computeIntersection(Forelimb fl, InteractionSurface is, ProcessPacket packet) {
    if (fl.numFingertips() <= 0) 
      return null;
    
    Point3f fingertip = new Point3f();
    for (Point3f p : fl.fingertipsW()) {
      fingertip.add(p);
    }
    fingertip.scale((float) 1 / fl.fingertipsW().size());
    
    Point3f armJoint = fl.armJointW();
    if (armJoint == null) {
      return null;
    }
    
    if (is.center().isNone()) {
      return null;
    }

    // Filtering
    filteredFingertip.updatePoints(new Point3f[] {fingertip});
    filteredArmjoint.updatePoints(new Point3f[] {armJoint});
    fingertip = filteredFingertip.getFilteredPoints()[0];
    armJoint = filteredArmjoint.getFilteredPoints()[0];
    
    // Find the subsampeled arm line.
    Point3f subsampledLinePoints[] = computeSubsampledLine(fingertip, armJoint, packet);
    Point3f p = Geometry.linePlaneIntersection(subsampledLinePoints[0], 
        subsampledLinePoints[1], 
        is.center().value(), is.surfaceNormal());

    
//    Point3f p = Geometry.linePlaneIntersection(armJoint, fingertip, 
//        is.center().value(), is.surfaceNormal());

    return new Point3D(p.x, p.y, p.z);
  }
  
  /**
   * Creates a line by subsampling between the armjoint and fingertip and
   * using those depth values in the line calculation. Returns the start and
   * endpoint of the subsampled line.
   * @param fingertip
   * @param armJoint
   * @param packet
   * @return
   */
  Point3f[] computeSubsampledLine(Point3f fingertip, Point3f armJoint,
      ProcessPacket packet) {
    float dist = fingertip.distance(armJoint);
    
    // Subsampled points going from fingertip to armjoint
    Point3f points[] = new Point3f[NUM_ARM_SUBSAMPLE_POINTS + 2];
    
    // Convert to projected coordinates for interpolation.
    try {
      points[0] = openni.convertRealWorldToProjective(fingertip);
      points[NUM_ARM_SUBSAMPLE_POINTS + 1] = openni.convertRealWorldToProjective(armJoint);
    } catch (StatusException e) {
      LOGGER.severe(e.getMessage());
    }    
    
    // Interpolate in between points
    for (int i = 0; i < NUM_ARM_SUBSAMPLE_POINTS; ++i) {
      float amount = (float)(i + 1) / ((float) NUM_ARM_SUBSAMPLE_POINTS + 2);
      points[i + 1] = Geometry.interpolate(points[0],
          points[NUM_ARM_SUBSAMPLE_POINTS + 1], amount);
    }
    
    CvMat pointMat = CvMat.create(1, points.length, CV_32FC3);
    ByteBuffer maskBuffer = packet.foregroundMask.getByteBuffer();
    int maskStepWidth = packet.foregroundMask.widthStep();
    
    // Do fingertip and armjoint.
    pointMat.put(0, fingertip.getX());
    pointMat.put(1, fingertip.getY());
    pointMat.put(2, fingertip.getZ());
    pointMat.put((NUM_ARM_SUBSAMPLE_POINTS+1) * 3, armJoint.getX());
    pointMat.put((NUM_ARM_SUBSAMPLE_POINTS+1) * 3 + 1, armJoint.getY());
    pointMat.put((NUM_ARM_SUBSAMPLE_POINTS+1) * 3 + 2, armJoint.getZ());
  
    for (int i = 1; i < points.length - 1; ++i) {
      
      try {
        Point3f p = findCentroid(packet, points[i]);
        if (p != null) {
          pointMat.put(i * 3, p.getX());
          pointMat.put(i * 3 + 1, p.getY());
          pointMat.put(i * 3 + 2, p.getZ());
        } else {
          p = points[i];
          p = openni.convertProjectiveToRealWorld(p);
          pointMat.put(i * 3, p.getX());
          pointMat.put(i * 3 + 1, p.getY());
          pointMat.put(i * 3 + 2, p.getZ());
        }
      } catch (StatusException e) {
        LOGGER.severe(e.getMessage());
      }    
      
    }
    
    // Fit line
    float[] armLine = new float[6];
    cvFitLine(pointMat, CV_DIST_L2, 0, 0.001, 0.001, armLine);
    pointMat.release();

    // Get line points
    Point3f subsampledLinePoints[] = new Point3f[2];
    subsampledLinePoints[0] = new Point3f(armLine[3], armLine[4], armLine[5]);
    subsampledLinePoints[1] = new Point3f(armLine[0], armLine[1], armLine[2]);
//    subsampledLinePoints[1].scale(dist);
    subsampledLinePoints[1].add(subsampledLinePoints[0]);
    
    return subsampledLinePoints;
  }
  
  /**
   * Finds the centroid of points in a rectangular region around a point.
   */
  private Point3f findCentroid(ProcessPacket packet, Point3f centerPoint) 
      throws StatusException {

    CvRect rect = new CvRect((int)centerPoint.x - RECT_RADIUS, (int)centerPoint.y
        - RECT_RADIUS, RECT_RADIUS * 2 + 1, RECT_RADIUS * 2 + 1);

    ByteBuffer maskBuffer = packet.foregroundMask.getByteBuffer();
    int maskStepWidth = packet.foregroundMask.widthStep();
    List<Point3D> list = new ArrayList<Point3D>();
    
    for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
      for (int x = rect.x(); x < rect.x() + rect.width(); x++) {
        if ((maskBuffer.get(y * maskStepWidth + x) & 0xff) == 255) {
          list.add(new Point3D(x, y,
              packet.depthRawData[y * packet.width + x]));
        }
      }
    }
    
    
    if (list.size() == 0) {
      return null;
    }

    Point3D[] points = new Point3D[list.size()];
    list.toArray(points);
    Point3D[] converted = openni.convertProjectiveToRealWorld(points);
    float centerx = 0, centery = 0, centerz = 0;
    for (Point3D point : converted) {
      centerx += point.getX();
      centery += point.getY();
      centerz += point.getZ();
    }
    
    Point3f res = new Point3f(centerx / list.size(), centery / list.size(), centerz
        / list.size());

    return res;
  }
}
