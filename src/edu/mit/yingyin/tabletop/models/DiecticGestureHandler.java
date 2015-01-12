package edu.mit.yingyin.tabletop.models;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3f;

import org.OpenNI.Point3D;

import edu.mit.yingyin.util.FilteredPoint3fs;
import edu.mit.yingyin.util.Geometry;

/**
 * Handles deictic gestures. Tasks this object performs include computing 
 * the point on the surface this hand is pointing at.
 * @author yingyin
 *
 */
public class DiecticGestureHandler {
  
  private int FILTER_HISTORY_LENGTH = 5;
  private FilteredPoint3fs filteredFingertip, filteredArmjoint;
  
  public DiecticGestureHandler() {
    filteredFingertip = new FilteredPoint3fs(FILTER_HISTORY_LENGTH);
    filteredArmjoint = new FilteredPoint3fs(FILTER_HISTORY_LENGTH);
  }
  
  /**
   * @param forelimbs
   * @return a list of intersections of diectic gestures.
   */
  public List<Point3D> update(List<Forelimb> forelimbs) {
    InteractionSurface is = InteractionSurface.instance();
    List<Point3D> res = new ArrayList<Point3D>();
    if (is == null)
      return res;
    
    for (Forelimb fl : forelimbs) {
      Point3D p = computeIntersection(fl, is);
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
  private Point3D computeIntersection(Forelimb fl, InteractionSurface is) {
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
    
    Point3f p = Geometry.linePlaneIntersection(armJoint, fingertip, 
        is.center().value(), is.surfaceNormal());
    return new Point3D(p.x, p.y, p.z);
  }
}
