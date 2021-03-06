package edu.mit.yingyin.util;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class FileUtil {

  public final static String JPEG = "jpeg";
  public final static String JPG = "jpg";
  public final static String GIF = "gif";
  public final static String TIFF = "tiff";
  public final static String TIF = "tif";
  public final static String PNG = "png";

	public static void copyFile(String src, String dst) throws IOException {
		InputStream in = new FileInputStream(src);
		OutputStream out = new FileOutputStream(dst);
		
		byte[] buf = new byte[1024];
		int len;
		while((len = in.read(buf)) > 0)
			out.write(buf, 0, len);
		
		in.close();
		out.close();
	}
	
	/**
   * Gets the extension of a file. If there is no extension, return null.
   */  
  public static String getExtension(File f) {
    String ext = null;
    String s = f.getName();
    int i = s.lastIndexOf('.');

    if (i > 0 &&  i < s.length() - 1) {
        ext = s.substring(i+1).toLowerCase();
    }
    return ext;
  }
  
  public static File setExtension(File f, String ext) {
  	String path = f.getPath();
  	return new File(setExtension(path, ext));
  }
  
  /**
   * Creates a new file name with the new extension.
   * 
   * @param fileName the original file name.
   * @param ext the new extension without '.'.
   * @return new file name.
   */
  public static String setExtension(String fileName, String ext) {
    int i = fileName.lastIndexOf('.');
    String newPath = "";
    if (i >= 0)
      newPath = fileName.substring(0, i + 1) + ext;
    else
      //if there is no extension, append the extension
      newPath = fileName + '.' + ext;
    return newPath;
  }
  
  /**
   * Returns a new string formed by joining the strings using File.separator.
   * @param strings
   * @return an empty string if strings is null.
   */
  public static String join(String... strings) {
    if (strings == null)
      return "";
    StringBuffer sb = new StringBuffer();
    for (int i = 0; i < strings.length - 1; i++) {
      sb.append(strings[i]);
      sb.append(File.separatorChar);
    }
    sb.append(strings[strings.length - 1]);
    return sb.toString();
  }
  
  public static String basename(String fileName) {
    String[] tokens = fileName.split(String.valueOf(File.separatorChar));
    return tokens[tokens.length - 1];
  }
  
  public static String basename(String fileName, String suffix) {
    fileName = FileUtil.basename(fileName);
    int index = fileName.lastIndexOf(suffix);
    return fileName.substring(0, index);
  }
}
