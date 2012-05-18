package edu.mit.yingyin.tabletop.models;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;

import edu.mit.yingyin.tabletop.models.HandTracker.FingerEvent.FingerEventType;

/**
 * <code>HandTracker</code> tracks hand events based on estimated hand model 
 * parameters.
 * @author yingyin
 *
 */
public class HandTracker {
  /**
   * The listener interface for recieving finger events.
   * @author yingyin
   *
   */
  public static interface IHandEventListener {
    public void fingerPressed(List<FingerEvent> feList);
  }
  
  /**
   * An event generated by a finger.
   * @author yingyin
   *
   */
  public static class FingerEvent {
    public enum FingerEventType {PRESSED, RELEASED};
    
    public int frameID;
    /**
     * Position on depth image and position on display.
     */
    public Point3f posImage;
    public Point2f posDisplay;
    public FingerEventType type;
    
    public FingerEvent(Point3f posImage, Point2f posDisplay,
        int frameID, FingerEventType type) { 
      this.posImage = posImage;
      this.posDisplay = posDisplay;
      this.frameID = frameID;
      this.type = type;
    }
    
    public String toString()
    {
      return String.format("Image positon = " + posImage + 
          " Display position = " + posDisplay + "\n");
    }
  }

  private static final int DEBOUNCE_COUNT = 3;

  private List<IHandEventListener> listeners = 
      new ArrayList<IHandEventListener>();
  
  /**
   * Reference to the table.
   */
  private Table table;
  /** Counts the duration of contact or noncontact. */
  private int pressedCounter = 0, releasedCounter = 0;
  /** True if finger is pressed, false otherwise. */
  private boolean pressed = false;
  private CalibrationExample calibExample;
  
  public HandTracker(CalibrationExample calibExample) {
    table = Table.instance();
    this.calibExample = calibExample;
  }
  
  /**
   * Updates forelimbs information and generates events.
   * @param forelimbs information for all the forlimbs detected.
   * @param frameID frame ID for the current update.
   */
  public void update(List<Forelimb> forelimbs, int frameID) {
    List<FingerEvent> fingerEventList = noFilter(forelimbs, frameID);
    for (IHandEventListener l : listeners) 
      l.fingerPressed(fingerEventList);
  }
  
  public void addListener(IHandEventListener l) {
    listeners.add(l);
  }

  public List<FingerEvent> noFilter(List<Forelimb> forelimbs, int frameID) {
    List<FingerEvent> fingerEventList = new ArrayList<FingerEvent>();
    for (Forelimb forelimb : forelimbs) 
      for (Point3f tip : forelimb.filteredFingertips)
        fingerEventList.add(createFingerEvent(tip, frameID, 
                                            FingerEventType.PRESSED));
    return fingerEventList;
  }

  /**
   * Filters out finger pressed events.
   * @param forelimbs
   * @param frameID
   * @return
   */
  public List<FingerEvent> filterPressed(List<Forelimb> forelimbs, 
                                         int frameID) {
    List<FingerEvent> fingerEventList = new ArrayList<FingerEvent>();
    for (Forelimb forelimb : forelimbs)
      for (Point3f tip : forelimb.filteredFingertips) {
        float tipDepth = tip.z + Hand.FINGER_THICKNESS; 
        boolean inContact = table.isInContact((int)tip.x, (int)tip.y, tipDepth);
        if (inContact) {
          pressedCounter++;
          releasedCounter = 0;
        } else {
          releasedCounter++;
          pressedCounter = 0;
        }
        if (pressedCounter == DEBOUNCE_COUNT && !pressed) {
          pressed = true;
          fingerEventList.add(createFingerEvent(tip, frameID, 
                                                FingerEventType.PRESSED));
        } else if (releasedCounter == DEBOUNCE_COUNT && pressed) {
          pressed = false;
          fingerEventList.add(createFingerEvent(tip, frameID, 
                                                FingerEventType.RELEASED));
        }
      }
    return fingerEventList;
  }
  
  private FingerEvent createFingerEvent(Point3f posImage, int frameID, 
      FingerEventType type) {
    return new FingerEvent(posImage, 
        calibExample.imageToDisplayCoords(posImage.x, posImage.y),
        frameID, type);
  }
  
}
