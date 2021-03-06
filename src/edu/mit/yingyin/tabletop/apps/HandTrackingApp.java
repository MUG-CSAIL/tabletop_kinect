package edu.mit.yingyin.tabletop.apps;

import java.awt.Point;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Properties;
import java.util.Scanner;
import java.util.logging.Logger;

import org.OpenNI.GeneralException;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.OptionBuilder;

import edu.mit.yingyin.tabletop.controllers.ProcessPacketController;
import edu.mit.yingyin.tabletop.models.EnvConstant;
import edu.mit.yingyin.tabletop.models.FeatureBuilder;
import edu.mit.yingyin.tabletop.models.FeatureWriter;
import edu.mit.yingyin.tabletop.models.OpenNIDevice;
import edu.mit.yingyin.tabletop.models.HandTracker.DiecticEvent;
import edu.mit.yingyin.tabletop.models.HandTracker.ManipulativeEvent;
import edu.mit.yingyin.tabletop.models.HandTrackingEngine;
import edu.mit.yingyin.tabletop.models.HandTrackingEngine.IHandEventListener;
import edu.mit.yingyin.tabletop.models.ProcessPacket;
import edu.mit.yingyin.util.CommandLineOptions;
import edu.mit.yingyin.util.FileUtil;
import edu.mit.yingyin.util.ObjectIO;

/**
 * Application that tracks the fingertips in data from an OpenNI device. Saves
 * fingertip events to file in the end.
 * 
 * @author yingyin
 * 
 */
public class HandTrackingApp extends KeyAdapter {

  /**
   * Listens to hand events.
   * 
   * @author yingyin
   * 
   */
  private class HandEventListener implements IHandEventListener {
    /**
     * List of finger events detected in a frame.
     */
    private List<List<ManipulativeEvent>> fingerEventList = 
        new ArrayList<List<ManipulativeEvent>>();

    @Override
    public void fingerPressed(List<ManipulativeEvent> feList) {
      if (packetController != null) {
        for (ManipulativeEvent fe : feList)
          packetController.drawCircle((int) fe.posImage.x, (int) fe.posImage.y);
      }
      fingerEventList.add(feList);
    }

    /**
     * Outputs finger events.
     * 
     * @param pw <code>PrinterWriter</code> for output.
     */
    public void toOutput(PrintWriter pw) {
      pw.println("# frame-id x y z x y z ...");
      for (List<ManipulativeEvent> list : fingerEventList) {
        if (list.isEmpty())
          continue;
        pw.print(list.get(0).frameID + " ");
        for (ManipulativeEvent fe : list) {
          pw.print(String.format("%d %d %d ", (int) fe.posImage.x,
              (int) fe.posImage.y, (int) fe.posImage.z));
        }
        pw.println();
      }
    }

    @Override
    public void fingerPointed(DiecticEvent de) {}
  }

  private static final Logger LOGGER = Logger.getLogger(
      HandTrackingApp.class.getName());
  
  /**
   * Application properties.
   */
  private static final String CLASSIFICATION_FILE_PROP = "classification-file";
  private static final String DESCRIPTOR_FILE_PROP = "descriptor-file";
  private static final String APP_PROPS = FileUtil.join(EnvConstant.CONFIG_DIR, 
      "fingertip-tracking.properties");
  private static final String DEFAULT_OPENNI_CONFIG_FILE = FileUtil.join(
      EnvConstant.CONFIG_DIR, "config.xml");
  private static final String DEFAULT_CALIB_FILE = FileUtil.join(
      EnvConstant.DATA_DIR, "calibration", "calibration.txt");
  private static final String TIME_FORMAT = "yyyy-MM-dd_HH-mm-SS";
  private static final String OUTPUT_EXTENSION = ".log";

  @SuppressWarnings("static-access")
  public static void main(String[] args) {
    Option mainDirOpt = OptionBuilder.withArgName("main directory").withLongOpt(
        "dir").hasArg().withDescription(
        "The main directory for input and output. "
            + "The configuration file should be in <dir>/config/folder."
            + "The default dir is the current directory.").create("d");
    CommandLineOptions.addOption(mainDirOpt);
    CommandLineOptions.parse(args);
    String mainDir = CommandLineOptions.getOptionValue("d", ".");
    new HandTrackingApp(mainDir);
  }

  private String mainDir, openniConfigFile, calibrationFile, labelFile, 
                 classificationFile;
  private HandTrackingEngine engine;
  private ProcessPacketController packetController;
  private HandEventListener handEventListener;
  private PrintWriter descriptorPrintWriter;
  private SimpleDateFormat dateFormat = new SimpleDateFormat(TIME_FORMAT);
  private boolean displayOn = true, saveFingertip = false;
  private boolean paused = false;
  private FeatureWriter featureWriter;

  @SuppressWarnings("unchecked")
  public HandTrackingApp(String mainDir) {
    LOGGER.info("java.library.path = "
        + System.getProperty("java.library.path"));

    this.mainDir = mainDir;
    processConfig();
    
    try {
      engine = new HandTrackingEngine(openniConfigFile, calibrationFile);
    } catch (GeneralException ge) {
      LOGGER.info("OpenNI config file = " + openniConfigFile);
      LOGGER.severe(ge.getMessage());
      System.exit(-1);
    }
    handEventListener = new HandEventListener();
    engine.addHandEventListener(handEventListener);

    if (displayOn) {
      try {
        HashMap<ProcessPacketController.Options, Object> options = 
            new HashMap<ProcessPacketController.Options, Object>();
        if (labelFile != null) 
          options.put(ProcessPacketController.Options.LABEL, 
              (HashMap<Integer, List<Point>>) ObjectIO.readObject(labelFile));
        if (classificationFile != null)
          options.put(ProcessPacketController.Options.CLASSIFICATION,
              readClassificationFile(classificationFile));
        
        packetController = new ProcessPacketController(engine.depthWidth(),
            engine.depthHeight(), options);
        engine.addHandEventListener(packetController);
        packetController.addKeyListener(this);

      } catch (IOException e) {
        System.err.println(e.getMessage());
        System.exit(-1);
      }
    }

    while (isRunning()) {
      if (isPaused())
        continue;
      step();
    }

    print();
    cleanUp();
    System.exit(0);
  }

  public void keyPressed(KeyEvent ke) {
    switch (ke.getKeyCode()) {
      case KeyEvent.VK_N:
        paused = true;
        step();
        break;
      case KeyEvent.VK_P:
        paused = !paused;
        break;
      case KeyEvent.VK_ESCAPE:
      case KeyEvent.VK_Q:
        packetController.hide();
        break;
      default:
        break;
    }
  }
  
  private void processConfig() {
    Properties config = new Properties();
    FileInputStream in = null;
    try {
      in = new FileInputStream(FileUtil.join(mainDir, APP_PROPS));
      config.load(in);
      in.close();
    } catch (FileNotFoundException fnfe) {
      LOGGER.info("congfig file not found: " + mainDir + APP_PROPS);
      LOGGER.severe(fnfe.getMessage());
      System.exit(-1);
    } catch (IOException ioe) {
      LOGGER.severe(ioe.getMessage());
      System.exit(-1);
    }

    // Processes configuration properties.
    openniConfigFile = FileUtil.join(mainDir,
        config.getProperty("openni-config", DEFAULT_OPENNI_CONFIG_FILE));
    String basename = EnvConstant.DEFAULT_BASENAME;
    if (OpenNIDevice.isRecordingFile(openniConfigFile)) {
      basename = FileUtil.basename(openniConfigFile, 
                                   OpenNIDevice.RECORDING_SUFFIX);
    }

    String saveFingertipProperty = config.getProperty("save-fingertip-data",
        "false");
    if (saveFingertipProperty.equalsIgnoreCase("true"))
      saveFingertip = true;

    labelFile = config.getProperty("fingertip-label-file", null);
    if (labelFile != null)
      labelFile = FileUtil.join(mainDir, EnvConstant.FINGERTIP_DIR, labelFile);

    String displayOnProperty = config.getProperty("display-on", "true");
    if (displayOnProperty.equals("false"))
      displayOn = false;

    calibrationFile = FileUtil.join(mainDir,
        config.getProperty("calibration-file", DEFAULT_CALIB_FILE));
    
    String descriptorFile = config.getProperty(DESCRIPTOR_FILE_PROP, null);
    if (descriptorFile != null) {
      descriptorFile = FileUtil.join(mainDir, EnvConstant.DESCRIPTOR_DIR, 
                                     descriptorFile);
      try {
        descriptorPrintWriter = new PrintWriter(new File(descriptorFile));
        LOGGER.info("descriptor file: " + descriptorFile);
      } catch (FileNotFoundException e) {
        LOGGER.severe(e.getMessage());
        System.exit(-1);
      }
    }
    
    classificationFile = config.getProperty(CLASSIFICATION_FILE_PROP, 
                                                   null);
    if (classificationFile != null) 
      classificationFile = FileUtil.join(mainDir, EnvConstant.DESCRIPTOR_DIR, 
                                         classificationFile);
    
    String imageWidthStr = config.getProperty("image-width", null);
    if (imageWidthStr != null) {
      FeatureBuilder.imageWidth(Integer.parseInt(imageWidthStr));
    }

    String saveFeatures = config.getProperty("save-features", "false");
    if (saveFeatures.equalsIgnoreCase("true")) {
      String dir = FileUtil.join(mainDir, EnvConstant.GESUTRE_DIR);
      (new File(dir)).mkdirs();
      String filename = FileUtil.join(dir, 
                        basename + "-" + FeatureBuilder.imageWidth() + 
                        EnvConstant.GESTURE_FEATURE_SUFFIX);
      featureWriter = new FeatureWriter(filename);
    }
  }
  
  private void cleanUp() {
    if (descriptorPrintWriter != null) {
      descriptorPrintWriter.close();
      LOGGER.info("Done writing descriptors.");
    }
    if (featureWriter != null) {
      featureWriter.close();
      LOGGER.info("Done writing features.");
    }
    engine.release();
    if (packetController != null)
      packetController.release();
  }
 
  /**
   * Gets one more frame.
   */
  private void step() {
    try {
      ProcessPacket packet = engine.step();

      if (packetController != null)
        packetController.show(packet);
      
      if (featureWriter != null)
        featureWriter.output(packet);
      
      packet.release();
    } catch (GeneralException ge) {
      LOGGER.severe(ge.getMessage());
      engine.release();
      System.exit(-1);
    }
  }
    
  /**
   * Prints finger events for evaluation.
   */
  private void print() {
    if (!saveFingertip)
      return;
    
    PrintWriter pw = null;
    try {
      Date date = new Date();
      String fingertipFile = FileUtil.join(mainDir, EnvConstant.FINGERTIP_DIR, 
          dateFormat.format(date) + OUTPUT_EXTENSION);
      pw = new PrintWriter(fingertipFile);
      handEventListener.toOutput(pw);
      LOGGER.info("Tracker controller output done.");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } finally {
      if (pw != null)
        pw.close();
    }
  }
  

  private boolean isRunning() {
    return ((packetController != null && 
        packetController.isVisible() || 
        displayOn == false) && 
        !engine.isDone());
  }

  private boolean isPaused() {
    return packetController != null && paused;
  }
  
  private HashMap<Integer, Integer> readClassificationFile(String filename) {
    HashMap<Integer, Integer> classfication = new HashMap<Integer, Integer>();
    Scanner scanner = null;
    try {
      scanner = new Scanner(new File(filename));
      scanner.useDelimiter("[,\\s]");
      while (scanner.hasNext()) {
        int frameID = scanner.nextInt();
        int classLabel = scanner.nextInt();
        classfication.put(frameID, classLabel);
      }
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } finally {
      if (scanner != null)
        scanner.close();
    }
    return classfication;
  }
}
