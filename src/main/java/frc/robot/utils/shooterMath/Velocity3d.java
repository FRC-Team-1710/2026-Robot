package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;

public class Velocity3d {
  private double v_x;
  private double v_y;
  private double v_z;

  public static final Velocity3d kZero = new Velocity3d();

  /** Default constructor initializes to zero velocity */
  public Velocity3d() {
    this(0, 0, 0);
  }

  /** Create a Velocity3d from 3d components */
  public Velocity3d(double v_x, double v_y, double v_z) {
    this.v_x = v_x;
    this.v_y = v_y;
    this.v_z = v_z;
  }

  /** Create a Velocity3d from a velocity magnitude and a Rotation3d direction */
  public Velocity3d(LinearVelocity velocity, Rotation3d rotation) {
    var mat = rotation.toMatrix(); // 3x3 matrix
    double dirX = mat.get(0, 0); // first column = x-axis direction in world frame
    double dirY = mat.get(1, 0);
    double dirZ = mat.get(2, 0);

    this.v_x = velocity.in(MetersPerSecond) * dirX;
    this.v_y = velocity.in(MetersPerSecond) * dirY;
    this.v_z = velocity.in(MetersPerSecond) * dirZ;
  }

  /** Returns the X component of the velocity. */
  public double getX() {
    return this.v_x;
  }

  /** Returns the Y component of the velocity. */
  public double getY() {
    return this.v_y;
  }

  /** Returns the Z component of the velocity. */
  public double getZ() {
    return this.v_z;
  }

  /** Sets the X component of the velocity. */
  public void setX(double v_x) {
    this.v_x = v_x;
  }

  /** Sets the Y component of the velocity. */
  public void setY(double v_y) {
    this.v_y = v_y;
  }

  /** Sets the Z component of the velocity. */
  public void setZ(double v_z) {
    this.v_z = v_z;
  }

  /** Adds the components of another Velocity3d to this one. */
  public Velocity3d plus(Velocity3d second_vector) {
    return new Velocity3d(
        this.v_x + second_vector.v_x, this.v_y + second_vector.v_y, this.v_z + second_vector.v_z);
  }

  /** Subtracts the components of another Velocity3d from this one. */
  public Velocity3d minus(Velocity3d second_vector) {
    return new Velocity3d(
        this.v_x - second_vector.v_x, this.v_y - second_vector.v_y, this.v_z - second_vector.v_z);
  }

  /** Returns the inverse of this Velocity3d. */
  public Velocity3d inverse() {
    return new Velocity3d(-this.v_x, -this.v_y, -this.v_z);
  }

  /** Returns the magnitude (norm) of this Velocity3d. */
  public double norm() {
    return Math.sqrt(this.v_x * this.v_x + this.v_y * this.v_y + this.v_z * this.v_z);
  }

  public static Velocity3d from(ChassisSpeeds fieldSpeeds) {
    return new Velocity3d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, 0);
  }
}
