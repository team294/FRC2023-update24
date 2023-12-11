package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTRESwerveModuleState {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.  Resulting state always has an angle that is
   * within +/-90 degrees of the currentAngle, without any wrapping.  For example, if
   * currentAngle = -550 deg, then the resulting state's angle will be between -640 and -460 deg.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate180Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
     * Given a reference angle (such as 999 deg), finds the closest equivalent value of newAngle (such
     * as 435 deg) that is within +/-180 deg of the reference angle (in this case, 1155 deg).  Works for 
     * positive and negative angles.
     * @param scopeReference Current angle, in degrees.  Angle can be any value (not limited to +/-180 deg).
     * @param newAngle Target angle, in degrees.  Angle can be any value (not limited to +/-180 deg).
     * @return Closest angle (in degrees) that is equivalent to newAngle but within +/-180 deg of scopeReference.
     */
    private static double placeInAppropriate180Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}
