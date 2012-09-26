package edu.mit.yingyin.tabletop.controllers;

import java.awt.Point;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.List;

import org.OpenNI.GeneralException;

import com.googlecode.javacv.cpp.opencv_core.IplImage;

import edu.mit.yingyin.tabletop.models.ProcessPacket;
import edu.mit.yingyin.tabletop.views.ProcessPacketView;
import edu.mit.yingyin.tabletop.views.ProcessPacketView.Toggles;
import edu.mit.yingyin.util.CvUtil;
import edu.mit.yingyin.util.FPSCounter;

/**
 * Controls the interaction on the visualization for the ProcessPacket.
 * @author yingyin
 *
 */
public class ProcessPacketController extends KeyAdapter implements MouseListener 
{
  public String derivativeSaveDir = "data/derivative/";

  private FPSCounter fpsCounter;
  private HashMap<Integer, List<Point>> allLabels;
  private ProcessPacketView packetView;
  private ProcessPacket packet;
  
  /**
   * Initializes the models and the view.
   * @param width
   * @param height
   */
  public ProcessPacketController(int width, int height, 
      HashMap<Integer, List<Point>> labels) {
    this.allLabels = labels;
    packetView = new ProcessPacketView(width, height);
    fpsCounter = new FPSCounter("Processed", packetView.analysisFrame());
    packetView.addKeyListener(this);
    packetView.addMouseListener(this);
  }
  
  public void showDepthImage(boolean show) {
    packetView.showDepthImage(show);
  }
  
  public void showDiagnosticImage(boolean show) {
    packetView.showDiagnosticImage(show);
  }
  
  public void keyPressed(KeyEvent ke) {
    switch (ke.getKeyCode()) {
    case KeyEvent.VK_B:
      packetView.toggle(Toggles.SHOW_BOUNDING_BOX);
      break;
    case KeyEvent.VK_D:
      packetView.toggle(Toggles.SHOW_CONVEXITY_DEFECTS);
      break;
    case KeyEvent.VK_F:
      // Shows all the detected fingertips.
      packetView.toggle(Toggles.SHOW_FINGERTIP);
      break;
    case KeyEvent.VK_H:
      packetView.toggle(Toggles.SHOW_HULL);
      break;
    case KeyEvent.VK_M:
      packetView.toggle(Toggles.SHOW_MORPHED);
      break;
    case KeyEvent.VK_L:
      packetView.toggle(Toggles.SHOW_LABELS);
      break;
    case KeyEvent.VK_S:
      PrintWriter pw = null;
      try {
        pw = new PrintWriter(
            String.format(derivativeSaveDir + "%03d", packet.depthFrameID));
        CvUtil.saveImage(pw, packet.derivative);
        System.out.println("Saved image.");
      } catch (FileNotFoundException e) {
        e.printStackTrace();
      } finally {
        if (pw != null)
          pw.close();
      }
      break;
    case KeyEvent.VK_R:
      // Showing RGB image.
      packetView.setToggle(Toggles.SHOW_RGB_IMAGE, true);
      break;
    default: 
      break;
    }
  }
  
  /**
   * Shows different visualizations of the ProcessPacket.
   * @param packet
   * @throws GeneralException 
   */
  public void show(ProcessPacket packet) throws GeneralException {
    this.packet = packet;
    packetView.update(packet, allLabels.get(packet.depthFrameID));
    fpsCounter.computeFPS();
  }
  
  /**
   * Releases memory.
   */
  public void release() {
    packetView.release();
  }
  
  public boolean isVisible() {
    return packetView.isVisible();
  }
  
  public void hide() {
    packetView.hide();
  }
  
  public void addKeyListener(KeyListener kl) {
    packetView.addKeyListener(kl);
  }
  
  /**
   * Draws a circle at (x, y).
   * @param x
   * @param y
   */
  public void drawCircle(int x, int y) {
    packetView.drawCircle(x, y);
  }
  
  public void showUI() {
    packetView.showUI();
  }

  @Override
  public void mouseClicked(MouseEvent me) {
    Point p = me.getPoint();
    IplImage image = packet.derivative;
    float value = image.getFloatBuffer().get(p.y * image.widthStep() / 4 + p.x);
    packetView.setStatus("x = " + p.x + " y = " + p.y + " value = " + value);
  }

  @Override
  public void mouseEntered(MouseEvent arg0) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void mouseExited(MouseEvent arg0) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void mousePressed(MouseEvent arg0) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void mouseReleased(MouseEvent arg0) {
    // TODO Auto-generated method stub
    
  }
}    