package edu.mit.yingyin.tabletop.models;

import java.nio.FloatBuffer;
import java.util.Arrays;

import com.googlecode.javacv.cpp.opencv_core.CvMat;

public class HandPoseDescriptor {
  private final int NUM_CIRCLES = 5;
  private final int NUM_SECTORS = 8;
  private final int NUM_DEPTH_SECTIONS = 5;
  private final float radius, radiusWidthInv, sectorWidthInv, depthWidthInv;
  private float minDepth, maxDepth;
  private float[][][] histogram = 
      new float[NUM_CIRCLES][NUM_SECTORS][NUM_DEPTH_SECTIONS]; 
  
  public HandPoseDescriptor(CvMat points) {
    radius = findRadius(points);
    radiusWidthInv = NUM_CIRCLES / radius;
    sectorWidthInv = (float) (NUM_SECTORS / Math.PI * 2);
    findDepthMinMax(points);
    depthWidthInv = NUM_DEPTH_SECTIONS / (maxDepth - minDepth);
    computeDescriptor(points);
  }

  public double radius() { return radius; }
  
  /**
   * 
   * @param points centered at the origin.
   * @return
   */
  private float findRadius(CvMat points) {
    return (float) Math.sqrt(points.rows()) * 2;
  }
  
  /**
   * 
   * @param points a matrix of float numbers. Each row is a 3 dimensional point.
   */
  private void findDepthMinMax(CvMat points) {
    FloatBuffer fb = points.getFloatBuffer();
    fb.rewind();
    minDepth = Float.POSITIVE_INFINITY;
    maxDepth = Float.NEGATIVE_INFINITY;
    for (int i = 0; i < points.rows(); i++) {
      float d = fb.get(i * 3 + 2);
      minDepth = Math.min(d, minDepth);
      maxDepth = Math.max(d, maxDepth);
    }
  }
  
  private void computeDescriptor(CvMat points) {
    float[] p = new float[3];
    FloatBuffer fb = points.getFloatBuffer();
    fb.rewind();
    int total = 0;
    Arrays.fill(histogram, 0);
    while (fb.remaining() > 0) {
      fb.get(p);
      float r = (float) Math.sqrt(p[0] * p[0] + p[1] * p[1]);
      if (r < radius) {
        // From -pi to pi.
        float theta = (float) (Math.atan2(p[1], p[0]) + Math.PI);
        int rIndex = (int) (r * radiusWidthInv); 
        int sIndex = (int) (theta * sectorWidthInv);
        int dIndex = (int) ((p[2] - minDepth) * depthWidthInv);
        histogram[rIndex][sIndex][dIndex]++;
        total++;
      }
    }
    for (int r = 0; r < NUM_CIRCLES; r++) {
      for (int s = 0; s < NUM_SECTORS; s++) {
        for (int d = 0; d < NUM_DEPTH_SECTIONS; d++) {
          histogram[r][s][d] /= total;
        }
      }
    }
  }
}
