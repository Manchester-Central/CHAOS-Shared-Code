package com.chaos131.vision;

import org.littletonrobotics.junction.AutoLog;

/**
 * A data structure recording targetting data. This is good for recording things like game piece
 * locations on the field. Note that the TargettingData is ambivalent about the number of targets
 * received, and so if many units are trying to be targetted then additional processing should be
 * done. Otherwise it will be difficult to distinguish corresponding AzEl locations.
 */
@AutoLog
public class NetworkTargettingData {
  /** Timestamps from NetworkTables, in microseconds */
  public long[] timestamps = new long[] {};

  /** Values from the NetworkTables topic setup from setPosePipeline() */
  public double[] Az = new double[] {};

  /** Values from the NetworkTables topic setup from setPosePipeline() */
  public double[] El = new double[] {};

  /** Resets the data structure to 0-length arrays */
  public void reset() {
    timestamps = new long[] {};
    Az = new double[] {};
    El = new double[] {};
  }

  /**
   * Sets the length of every array to a specific size
   *
   * @param s size to set to
   */
  public void resize(int s) {
    timestamps = new long[s];
    Az = new double[s];
    El = new double[s];
  }
}
