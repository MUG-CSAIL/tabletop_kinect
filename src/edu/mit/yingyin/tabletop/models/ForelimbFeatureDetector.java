package edu.mit.yingyin.tabletop.models;

import static com.googlecode.javacv.cpp.opencv_core.CV_32SC1;
import static com.googlecode.javacv.cpp.opencv_core.CV_32SC2;
import static com.googlecode.javacv.cpp.opencv_core.CV_WHOLE_SEQ;
import static com.googlecode.javacv.cpp.opencv_core.IPL_DEPTH_8U;
import static com.googlecode.javacv.cpp.opencv_core.cvCopy;
import static com.googlecode.javacv.cpp.opencv_core.cvCreateMat;
import static com.googlecode.javacv.cpp.opencv_core.cvCvtSeqToArray;
import static com.googlecode.javacv.cpp.opencv_core.cvMat;
import static com.googlecode.javacv.cpp.opencv_core.cvRect;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_CHAIN_APPROX_SIMPLE;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_CLOCKWISE;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_MOP_OPEN;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_POLY_APPROX_DP;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_RETR_EXTERNAL;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_GAUSSIAN;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvApproxPoly;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvBoundingRect;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvContourPerimeter;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvConvexHull2;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvConvexityDefects;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvFindNextContour;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvMorphologyEx;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvSobel;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvStartFindContours;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvSmooth;

import java.nio.ByteBuffer;
import java.util.logging.Logger;

import org.OpenNI.StatusException;

import com.googlecode.javacpp.Loader;
import com.googlecode.javacv.cpp.opencv_core.CvContour;
import com.googlecode.javacv.cpp.opencv_core.CvPoint;
import com.googlecode.javacv.cpp.opencv_core.CvRect;
import com.googlecode.javacv.cpp.opencv_core.CvSeq;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc.CvContourScanner;

import edu.mit.yingyin.tabletop.models.ProcessPacket.ForelimbFeatures;
import edu.mit.yingyin.util.CvUtil;

/**
 * HandAnalyzer estimates the parameters of the hand model using measurements
 * from the video images.
 * 
 * @author yingyin
 * 
 */
public class ForelimbFeatureDetector {
  private static Logger LOGGER = Logger.getLogger(
      ForelimbFeatureDetector.class.getName());

  /**
   * Number of initial frames to ignore.
   */
  private static final int BG_INGNORE_FRAMES = 20;

  /**
   * Number of initial frames to initialize the background.
   */
  private static final int BG_INIT_FRAMES = BG_INGNORE_FRAMES + 40;

  private static final float BG_DIFF_LSCALE = 5;
  private static final float BG_DIFF_HSCALE = 6;

  /**
   * The number of iterations of morphological transformation.
   */
  private static final int MORPH_ITR = 1;
  private static final int CONTOUR_APPROX_LEVEL = 2;
  /**
   * The ratio between the perimeter of the table and the perimeter of the hand.
   * Assumes a fully extended hand's dimension is w = 15cm, h = 15cm, and the 
   * table's dimension is w = 122cm, h = 92cm.
   */
  private static final int HAND_PERIM_SCALE = 7;
  /**
   * The ratio between the height of the table and the maximum (fully extended)
   * height of the hand. 
   */
  private static final int HAND_MAX_HEIGHT_SCALE = 10;
  /**
   * The ratio between the height of the table and the minimum height of the
   * hand. The minimum hand height is assumed to be 8cm. 
   */
  private static final int HAND_MIN_HEIGHT_SCALE = 13;
  private static final int ARM_JOINT_HEIGHT_SCALE = 13;

  private static final int FORELIMB_BOTTOM_TO_IMAGE_BOTTOM_DIST_THRESH = 10;
  
  private static final boolean HAND_ORIENTATION_UP_FROM_BOTTOM = true;
  
  private Background background;
  private final IplImage tempImage;
  private final ForelimbModelEstimator forelimbModelEstimator;
  private final OpenNIDevice openni;
  private final HandFeatureDetector hpfd;
  
  private int lastDepthFrameID;
  
  /**
   * Initializes the data structures.
   * 
   * @param width
   * @param height
   * @param egnine reference to the <code>HandTrackingEngine</code>.
   */
  public ForelimbFeatureDetector(int width, int height, OpenNIDevice openni) {
    tempImage = IplImage.create(width, height, IPL_DEPTH_8U, 1);
    background = Background.initInstance(width, height);
    forelimbModelEstimator = new ForelimbModelEstimator(width, height, openni);
    this.openni = openni;
    hpfd = new HandFeatureDetector(width, height, openni);
    lastDepthFrameID = 0;
  }

  /**
   * Hand data analysis pipeline.
   * 
   * @param packet contains all the relevant data for analysis.
   * @throws StatusException
   */
  public void detect(ProcessPacket packet) throws StatusException {
    packet.clear();
    
    lastDepthFrameID = packet.depthFrameID;


    if (packet.depthFrameID < BG_INGNORE_FRAMES)
      return;

    CvUtil.intToIplImage32F(packet.depthRawData, packet.depthImageBlur32F, 
        1);
    cvSmooth(packet.depthImageBlur32F, packet.depthImageBlur32F, CV_GAUSSIAN, 5);
    if (packet.depthFrameID < BG_INIT_FRAMES) {
      background.accumulateBackground(packet.depthRawData);
      return;
    } else if (packet.depthFrameID == BG_INIT_FRAMES) {
      background.createModelsFromStats((float) BG_DIFF_LSCALE,
          (float) BG_DIFF_HSCALE);
      InteractionSurface.initInstance(background, openni);
      LOGGER.info(background.stats());
    }

    CvUtil.intToIplImage32F(packet.depthRawData, packet.depthImage32F, 
        (float) 1 / background.maxDepth());
    cvSobel(packet.depthImage32F, packet.derivative, 2, 2, 3);

    subtractBackground(packet);
    cleanUpBackground(packet);
    findConnectedComponents(packet, HAND_PERIM_SCALE);
    findHandRegions(packet);
    hpfd.detect(packet);
    forelimbModelEstimator.updateModel(packet);
    
  }

  /**
   * Releases memory.
   */
  public void release() {
    tempImage.release();
    background.release();
    LOGGER.info("HandAnalyzer released.");
  }
  
  public void recalibrateBackground() {
    background = Background.resetInstance();
    InteractionSurface.clearInstancve();
    LOGGER.info("Background recalibrating.");
  }
  
  /**
   * Returns true whenever recording frames to subtract from the background.
   * @return
   */
  public boolean isCalibratingBackground() {
    return lastDepthFrameID < BG_INIT_FRAMES;
  }
  

  protected void subtractBackground(ProcessPacket packet) {
    int[] depthData = packet.depthRawData;
    IplImage depthImage = packet.depthImage8U;

    background.backgroundDiff(packet.depthRawData, packet.foregroundMask);
    ByteBuffer depthBuffer = depthImage.getByteBuffer();
    ByteBuffer maskBuffer = packet.foregroundMask.getByteBuffer();
    int maskWidthStep = packet.foregroundMask.widthStep();
    int depthWidthStep = depthImage.widthStep();
    int width = packet.width;
    for (int h = 0; h < packet.height; h++)
      for (int w = 0; w < packet.width; w++) {
        int pos = h * depthWidthStep + w;
        if (Background.isForeground(maskBuffer.get(h * maskWidthStep + w))) {
          byte d = (byte) (depthData[h * width + w] * 255 / 
                           background.maxDepth());
          depthBuffer.put(pos, d);
        } else {
          depthBuffer.put(pos, (byte) 0);
        }
      }
  }

  /**
   * Cleans up the background subtracted image.
   * 
   * @param packet ProcessPacket containing the data.
   */
  private void cleanUpBackground(ProcessPacket packet) {
    // The default 3x3 kernel with the anchor at the center is used.
    // The opening operator involves erosion followed by dilation. Its effect is
    // to eliminate lone outliers that are higher in intensity (bumps) than
    // their neighbors.
    cvMorphologyEx(packet.depthImage8U, packet.morphedImage, null, null,
        CV_MOP_OPEN, MORPH_ITR);
  }

  /**
   * Finds connected components as forelimbs.
   * 
   * @param packet ProcessPacket containing the data necessary for the analysis.
   * @param perimScale len = (image.width + image.height) / perimScale. If
   *          contour length < len, delete that contour.
   * @return a sequence of contours
   */
  private void findConnectedComponents(ProcessPacket packet, float perimScale) {
    cvCopy(packet.morphedImage, tempImage);

    // CV_RETR_EXTERNAL: retrieves only the extreme outer contours.
    // CV_CHAIN_APPROX_SIMPLE: compresses horizontal, vertical, and diagonal
    // segments, leaving only their ending points.
    CvContourScanner scanner =
        cvStartFindContours(tempImage, packet.tempMem,
            Loader.sizeof(CvContour.class), CV_RETR_EXTERNAL,
            CV_CHAIN_APPROX_SIMPLE);
    CvSeq c;
    double q =
        (packet.morphedImage.height() + packet.morphedImage.width()) /
            perimScale;
    while ((c = cvFindNextContour(scanner)) != null) {
      double len = cvContourPerimeter(c);
      if (len > q) {
        ForelimbFeatures ff = new ForelimbFeatures();
        // Approximates the contour with fewer vertices. Only CV_POLY_APPROX_DP
        // is supported which corresponds to Douglas-Peucker algorithm.
        CvSeq approxPoly =
            cvApproxPoly(c, Loader.sizeof(CvContour.class), packet.tempMem,
                CV_POLY_APPROX_DP, CONTOUR_APPROX_LEVEL, 0);
        CvPoint approxPolyPts = new CvPoint(approxPoly.total());
        cvCvtSeqToArray(approxPoly, approxPolyPts, CV_WHOLE_SEQ);
        ff.approxPoly = cvMat(1, approxPoly.total(), CV_32SC2, approxPolyPts);
        ff.boundingBox = cvBoundingRect(ff.approxPoly, 0);
        ff.hull = cvCreateMat(1, approxPoly.total(), CV_32SC1);
        // returnPoints = 0: returns pointers to the points in the contour
        cvConvexHull2(ff.approxPoly, ff.hull, CV_CLOCKWISE, 0);
        ff.convexityDefects =
            cvConvexityDefects(ff.approxPoly, ff.hull, packet.tempMem);
        packet.forelimbFeatures.add(ff);
      }
    }
  }

  /**
   * Finds rectangle hand regions from forelimb regions.
   * 
   * @param packet contains all the processing information.
   */
  private void findHandRegions(ProcessPacket packet) {
    int maxHandHeight = packet.height / HAND_MAX_HEIGHT_SCALE;
    int minHandHeight = packet.height / HAND_MIN_HEIGHT_SCALE;
    int armJointHeight = packet.height / ARM_JOINT_HEIGHT_SCALE;

    if (HAND_ORIENTATION_UP_FROM_BOTTOM) {
      for (ForelimbFeatures ff : packet.forelimbFeatures) {
        CvRect rect = ff.boundingBox;
        if (rect.height() < maxHandHeight) {
          maxHandHeight = rect.height();
        }
        if (rect.height() >= minHandHeight) {
          int y1 = rect.y();
          int y2 = y1 + rect.height();
          
          int yhand = y1, yarm = y2 - armJointHeight;
          ff.handRegion = cvRect(rect.x(), yhand, rect.width(), maxHandHeight);
          
          if (rect.height() - maxHandHeight >= armJointHeight) {
            ff.armJointRegion =
                cvRect(rect.x(), yarm, rect.width(), armJointHeight);
          }
        }
      }
      
    } else {
      
      for (ForelimbFeatures ff : packet.forelimbFeatures) {
        CvRect rect = ff.boundingBox;
        if (rect.height() < maxHandHeight)
          maxHandHeight = rect.height();
        if (rect.height() >= minHandHeight) {
          int y1 = rect.y();
          int y2 = y1 + rect.height();
          int yhand = y1, yarm = y2 - armJointHeight;
          if (!isForelimbAtBottom(y2, packet.height)) {
            yhand = y2 - maxHandHeight;
            yarm = y1;
          }
          ff.handRegion = cvRect(rect.x(), yhand, rect.width(), maxHandHeight);
          if (rect.height() - maxHandHeight >= armJointHeight) {
            ff.armJointRegion =
                cvRect(rect.x(), yarm, rect.width(), armJointHeight);
          }
        }
      }
    }
  }
  
  private boolean isForelimbAtBottom(int forelimbBottom, int imageBottom) {
    return Math.abs(forelimbBottom - imageBottom) < 
           FORELIMB_BOTTOM_TO_IMAGE_BOTTOM_DIST_THRESH;
  }
}
