package edu.mit.yingyin.tabletop.app;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import org.apache.commons.cli.Option;

import edu.mit.yingyin.gui.ImageComponent;
import edu.mit.yingyin.gui.ImageFrame;
import edu.mit.yingyin.util.CommandLineOptions;

public class RawFileConverter {
  private static final String USAGE = "Usage: Arguments: <input file name> " +
  		"[type] [width] [height]";
  private static final String DEFAULT_WIDTH = "640";
  private static final String DEFAULT_HEIGHT = "480";
  private static final String DEFAULT_TYPE = "rgb";
  private static String inputFile;
  
  public static void main(String[] args) {
    // Command line processing.
    if (args.length < 1) {
      System.out.println(USAGE);
      System.exit(-1);
    } else { 
      inputFile = args[0];
    }
    
    CommandLineOptions.addOption(
        new Option("w", true, "Width of the image"));
    CommandLineOptions.addOption(
        new Option("h", true, "Height of the image"));
    CommandLineOptions.addOption(
        new Option("t", true, "Type of the image"));
    
    CommandLineOptions.parse(args);
    
    int width = Integer.parseInt(
        CommandLineOptions.getOptionValue("w", DEFAULT_WIDTH));
    int height = Integer.parseInt(
        CommandLineOptions.getOptionValue("h", DEFAULT_HEIGHT));
    String type = CommandLineOptions.getOptionValue("t", DEFAULT_TYPE);
    
    int bytesPerPixel = 3;
    int bufferedImageType = BufferedImage.TYPE_3BYTE_BGR;
    if (type == "depth") {
      bytesPerPixel = 1;
      bufferedImageType = BufferedImage.TYPE_USHORT_GRAY;
    }
    
    try {
      BufferedInputStream bis = new BufferedInputStream(
          new FileInputStream(inputFile));
      int length = width * height * bytesPerPixel;
      byte[] buffer = new byte[length];
      bis.read(buffer, 0, length);
      BufferedImage bi = new BufferedImage(width, height, bufferedImageType);
      byte[] dstArray = ((DataBufferByte)bi.getRaster().getDataBuffer()).
          getData();
      for (int i = 0; i < width * height; i++) 
        for (int j = 0; j < bytesPerPixel; j++) {
          dstArray[i * bytesPerPixel + j] = buffer[(i + 1) * bytesPerPixel - 1 
                                                   - j];
        }
        
      ImageComponent ic = new ImageComponent(new Dimension(width, height));
      ic.setImage(bi);
      new ImageFrame("image", ic);
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
