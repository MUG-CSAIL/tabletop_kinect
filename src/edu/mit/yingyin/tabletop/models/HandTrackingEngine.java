package edu.mit.yingyin.tabletop.models;

import java.util.List;
import java.util.logging.Logger;

import org.OpenNI.GeneralException;

import edu.mit.yingyin.calib.CalibModel;
import edu.mit.yingyin.tabletop.models.HandTracker.DiecticEvent;
import edu.mit.yingyin.tabletop.models.HandTracker.ManipulativeEvent;

/**
 * Main interface to the hand tracking back-end module that tracks the hand
 * events and updates the hand event listeners with the events.
 * 
 * @author yingyin
 * 
 */
public class HandTrackingEngine {
  /**
   * The listener interface for receiving finger events.
   * 
   * @author yingyin
   * 
   */
  public static interface IHandEventListener {
    public void fingerPressed(List<ManipulativeEvent> feList);

    public void fingerPointed(DiecticEvent de);
  }

  private static Logger logger =
      Logger.getLogger(HandTrackingEngine.class.getName());

  private final OpenNIDevice openni;
  private int depthWidth, depthHeight;
  private int prevDepthFrameID = -1, currentDepthFrameID = -1;
  private HandTracker tracker;
  private ForelimbFeatureDetector featureDetector;
  
  private int depthFrameIDOffset;
  
  /**
   * Helper for debugging purposes. Flips the depth image when using 
   * Kinect connected to Linux machine.
   */
  private static final boolean FLIP_DEPTH_BASED_ON_OS = true;

  /**
   * Creates a new <code>HandTrackingEngine</code>.
   * 
   * @param openniConfigFile
   * @param calibrationFile
   * @throws GeneralException
   */
  public HandTrackingEngine(String openniConfigFile, String calibrationFile)
      throws GeneralException {

    openni = new OpenNIDevice(openniConfigFile);

    depthWidth = openni.getDepthWidth();
    depthHeight = openni.getDepthHeight();
    featureDetector =
        new ForelimbFeatureDetector(depthWidth, depthHeight, openni);

    tracker = new HandTracker(new CalibModel(calibrationFile), openni);
    
    this.depthFrameIDOffset = 0;
  }
  
  public void resetDepthFrameID() {
    this.depthFrameIDOffset = this.currentDepthFrameID;
  }
  
  public int getDepthFrameID() {
    this.currentDepthFrameID = openni.getDepthFrameID();
    return this.currentDepthFrameID - this.depthFrameIDOffset;
  }

  public int depthWidth() {
    return depthWidth;
  }

  public int depthHeight() {
    return depthHeight;
  }

  public void release() {
    openni.release();
    featureDetector.release();
  }

  public void addHandEventListener(IHandEventListener l) {
    tracker.addListener(l);
  }

  public void removeListener(IHandEventListener l) {
    tracker.removeListener(l);
  }

  /**
   * Checks if all the frames are played.
   * 
   * @return true if all the frames are played.
   */
  public boolean isDone() {
    return currentDepthFrameID < prevDepthFrameID;
  }

  /**
   * Steps one frame.
   * 
   * @return a new {@code ProcessPacket}. The caller should release the memory
   *         of the returned object when finished using the object.
   */
  public ProcessPacket step() {
    ProcessPacket packet = null;
    try {
      openni.waitDepthUpdateAll();
      // New packet for each frame.
      packet = new ProcessPacket(depthWidth, depthHeight, openni);
      openni.getDepthArray(packet.depthRawData);
      prevDepthFrameID = currentDepthFrameID;
      packet.depthFrameID = this.getDepthFrameID();
      //currentDepthFrameID = packet.depthFrameID;

      if (FLIP_DEPTH_BASED_ON_OS) {
        if (System.getProperty("os.name").equals("Linux")) {
          flipPacketDepth(packet);
        }
      }
      
      featureDetector.detect(packet);

      if (interactionSurfaceInitialized())
        tracker.update(packet.forelimbs, packet.depthFrameID, packet);
    } catch (Exception e) {
      logger.severe(e.getMessage());
      e.printStackTrace();
      System.exit(-1);
    }
    return packet;
  }
  
  /**
   * Flips the depth array of a ProcessPacket
   * @param packet
   */
  public void flipPacketDepth(ProcessPacket packet) {
    int[] depthArray = packet.depthRawData;
    
//    int[] tempRow = new int[packet.width];
//    
//    for (int h = 0; h < packet.height/2; h++) {
//      int srcPos1 = h * packet.width;
//      int srcPos2 = depthArray.length - (h + 1) * packet.width;
//      System.arraycopy(depthArray, srcPos1, tempRow, 0, packet.width);
//      System.arraycopy(depthArray, srcPos2, depthArray, srcPos1, packet.width);
//      System.arraycopy(tempRow, 0, depthArray, srcPos2, packet.width);
//    }
    
    int temp, ii;
    for(int i = 0; i < depthArray.length/2; ++i) {
      ii = depthArray.length - i - 1;
      temp = depthArray[i];
      depthArray[i] = depthArray[ii];
      depthArray[ii] = temp;
    }
    
  }
  
  /**
   * Resets the background calibration of the tabletop.
   */
  public void recalibrateBackground() {
    this.resetDepthFrameID();
    featureDetector.recalibrateBackground();
  }
  
  /**
   * Returns true whenever recording frames to subtract from the background.
   * @return
   */
  public boolean isCalibratingBackground() {
    return featureDetector.isCalibratingBackground();
  }

  public boolean interactionSurfaceInitialized() {
    return InteractionSurface.instanceInitialized();
  }

  public InteractionSurface interactionSurface() {
    return InteractionSurface.instance();
  }
}