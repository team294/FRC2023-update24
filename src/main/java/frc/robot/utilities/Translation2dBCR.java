// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dBCR {
  /**
   * Returns the dot product of two translations in 2D space.
   *
   * <p>For example, Translation3d(1.0, 2.0) dot Translation3d(2.5, 5.5) = 2.5 + 11.0 = 13.5.
   *
   * @param t1 The first translation.
   * @param t2 The second translation.
   * @return The dot product of the translations.
   */
  public static double dot(Translation2d t1, Translation2d t2) {
    return t1.getX()*t2.getX() + t1.getY()*t2.getY();
  }

  /**
   * Returns the normalized version of a translations in 2D space.  The normalized version
   * is a vector in the same direction as the original vector but with length = 1.0.
   *
   * @param t1 The translation to normalize.
   * @return The normalized translation.
   */
  public static Translation2d normalize(Translation2d t1) {
    double len = t1.getNorm();
    if (len==0) {
      return t1;
    }
    return t1.div(len);    
  }
}
