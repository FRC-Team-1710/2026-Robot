package frc.robot.utils.ShooterMath;

import edu.wpi.first.math.geometry.Rotation2d;

public class Velocity2d {
  private double v_x;
  private double v_y;

  public static final Velocity2d kZero = new Velocity2d();

  /** Default constructor initializes velocity to zero. */
  public Velocity2d() {
    this(0, 0);
  }

  /** Constructor initializes velocity to given values. */
  public Velocity2d(double v_x, double v_y) {
    this.v_x = v_x;
    this.v_y = v_y;
  }

  /** Create a Velocity2d from a velocity magnitude and an angle in radians. */
  public Velocity2d(double velocity, Rotation2d angle) {
    this.v_x = velocity * angle.getCos();
    this.v_y = velocity * angle.getSin();
  }

  /** Returns the X component of the velocity. */
  public double getX() {
    return this.v_x;
  }

  /** Returns the Y component of the velocity. */
  public double getY() {
    return this.v_y;
  }

  /** Sets the X component of the velocity. */
  public void setX(double v_x) {
    this.v_x = v_x;
  }

  /** Sets the Y component of the velocity. */
  public void setY(double v_y) {
    this.v_y = v_y;
  }

  /** Adds two Velocity2d vectors. */
  public Velocity2d plus(Velocity2d second_vector) {
    return new Velocity2d(this.v_x + second_vector.v_x, this.v_y + second_vector.v_y);
  }

  /** Subtracts one Velocity2d vector from another. */
  public Velocity2d minus(Velocity2d second_vector) {
    return new Velocity2d(this.v_x - second_vector.v_x, this.v_y - second_vector.v_y);
  }

  /** Returns the inverse of the Velocity2d vector. */
  public Velocity2d inverse() {
    return new Velocity2d(-this.v_x, -this.v_y);
  }

  /** Returns the magnitude (norm) of the Velocity2d vector. */
  public double norm() {
    return Math.sqrt(this.v_x * this.v_x + this.v_y * this.v_y);
  }

  /** Creates a Velocity2d from a Velocity3d by dropping the Z component. */
  public Velocity2d from(Velocity3d velocity3d) {
    return new Velocity2d(velocity3d.getX(), velocity3d.getY());
  }
}
