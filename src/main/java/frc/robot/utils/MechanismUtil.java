package frc.robot.utils;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Utility class for creating and managing Mechanism2d visualizations.
 *
 * <p>This class provides helper classes for common mechanism visualizations used in robot
 * simulation, including arms and flywheels.
 */
public class MechanismUtil {

  /**
   * Helper class for creating and updating an arm mechanism visualization.
   *
   * <p>This class encapsulates the Mechanism2d visualization for a single-jointed arm, including
   * the base, tower, pivot, and arm ligaments. It provides a simple interface for updating the
   * arm's angle and color based on its state.
   */
  public static class ArmMechanism {
    // ==================== Visualization Constants ====================

    /** Width of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_WIDTH = 400.0;

    /** Height of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_HEIGHT = 400.0;

    /** X position of the mechanism root on the canvas */
    private static final double ROOT_X = 200.0;

    /** Y position of the mechanism root on the canvas */
    private static final double ROOT_Y = 0;

    /** Width of the base structure in pixels */
    private static final double BASE_WIDTH = 40.0;

    /** Height of the base structure in pixels */
    private static final double BASE_HEIGHT = 20.0;

    /** Height of the tower structure in pixels */
    private static final double TOWER_HEIGHT = 30.0;

    /** Visual width of the arm ligament in pixels */
    private static final double ARM_WIDTH = 10.0;

    /** Visual size of the pivot point in pixels */
    private static final double PIVOT_SIZE = 5.0;

    /** Color of arm when not at target position */
    private static final Color8Bit MOVING_COLOR = new Color8Bit(Color.kYellow);

    /** Color of arm when at target position */
    private static final Color8Bit AT_TARGET_COLOR = new Color8Bit(Color.kGreen);

    /**
     * Angle offset to convert from unit circle coordinates to mechanism ligament coordinates.
     *
     * <p>The arm ligament is attached to a pivot at 90° (on top of vertical tower), but encoder
     * angles follow unit circle convention (0° = right, 90° = up). This offset converts between the
     * two coordinate systems.
     */
    private static final double ANGLE_OFFSET = 90.0;

    // ==================== Visualization Components ====================

    /** 2D mechanism visualization */
    private final Mechanism2d mech;

    /** Visual representation of the arm that updates with simulation */
    private final MechanismLigament2d armLigament;

    /**
     * Constructs a new ArmMechanism visualization.
     *
     * @param name The name to use for the mechanism visualization
     * @param armVisualLength The length of the arm in pixels
     */
    public ArmMechanism(String name, double armVisualLength) {
      // Create the 2D mechanism visualization canvas
      mech = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
      MechanismRoot2d root = mech.getRoot(name + "Root", ROOT_X, ROOT_Y);

      // Build the visual hierarchy: Base -> Tower -> Pivot -> Arm

      // Base platform (horizontal, dark gray)
      MechanismLigament2d armBase =
          root.append(
              new MechanismLigament2d(
                  "Base", BASE_WIDTH, 0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

      // Tower extending upward from base (vertical, gray)
      MechanismLigament2d tower =
          armBase.append(
              new MechanismLigament2d(
                  "Tower", TOWER_HEIGHT, 90, BASE_HEIGHT / 2, new Color8Bit(Color.kGray)));

      // Pivot point at top of tower (small black circle)
      MechanismLigament2d pivot =
          tower.append(
              new MechanismLigament2d(
                  "Pivot", PIVOT_SIZE, 0, PIVOT_SIZE, new Color8Bit(Color.kBlack)));

      // The arm itself (starts yellow, will turn green when at target)
      armLigament =
          pivot.append(new MechanismLigament2d("Arm", armVisualLength, 0, ARM_WIDTH, MOVING_COLOR));
    }

    /**
     * Gets the Mechanism2d object for publishing to SmartDashboard.
     *
     * @return The Mechanism2d visualization
     */
    public Mechanism2d getMechanism() {
      return mech;
    }

    /**
     * Updates the visual representation of the arm.
     *
     * @param angleDeg The current angle of the arm in degrees (unit circle convention: 0° = right,
     *     90° = up)
     * @param atTarget Whether the arm is at its target position
     */
    public void update(double angleDeg, boolean atTarget) {
      // Update the visual representation of the arm
      // The arm ligament is relative to the pivot, which sits at 90° absolute (on top
      // of vertical
      // tower)
      // Encoder angle is absolute (0° = right, 90° = up per unit circle)
      // So subtract 90° to convert: encoder 0° → visual -90° (relative to pivot) = 0°
      // absolute
      // (right)
      armLigament.setAngle(angleDeg - ANGLE_OFFSET);

      // Change color based on whether at target position (green = ready, yellow =
      // moving)
      Color8Bit currentColor = atTarget ? AT_TARGET_COLOR : MOVING_COLOR;
      armLigament.setColor(currentColor);
    }
  }

  /**
   * Helper class for creating and updating a flywheel mechanism visualization.
   *
   * <p>This class encapsulates the Mechanism2d visualization for a flywheel with rotating spokes.
   * It provides a simple interface for updating the flywheel's rotation based on its velocity and
   * changing color based on whether it's at target speed.
   */
  public static class FlywheelMechanism {
    // ==================== Visualization Constants ====================

    /** Width of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_WIDTH = 400.0;

    /** Height of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_HEIGHT = 400.0;

    /** X position of the mechanism root on the canvas */
    private static final double ROOT_X = 200.0;

    /** Y position of the mechanism root on the canvas */
    private static final double ROOT_Y = 0.5;

    /** Width of each spoke in the flywheel visualization in pixels */
    private static final double SPOKE_WIDTH = 2.0;

    /** Number of spokes in the flywheel visualization */
    private static final int NUM_SPOKES = 16;

    /** Color of flywheel when not spinning or slow */
    private static final Color8Bit IDLE_COLOR = new Color8Bit(Color.kRed);

    /** Color of flywheel when spinning at target speed */
    private static final Color8Bit ACTIVE_COLOR = new Color8Bit(Color.kGreen);

    // ==================== Visualization Components ====================

    /** 2D mechanism visualization */
    private final Mechanism2d mech;

    /** Visual representation of the flywheel spokes that rotate with simulation */
    private final MechanismLigament2d[] spokes;

    /** Current rotation angle for visual animation in degrees */
    private double visualAngleDeg = 0.0;

    /**
     * Constructs a new FlywheelMechanism visualization.
     *
     * @param name The name to use for the mechanism visualization
     * @param flywheelRadius The visual radius of the flywheel in pixels
     */
    public FlywheelMechanism(String name, double flywheelRadius, double offset) {
      // Create the 2D mechanism visualization canvas
      mech = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
      MechanismRoot2d root = mech.getRoot(name + "Root", ROOT_X, ROOT_Y + offset);

      // Build the visual representation: multiple spokes radiating from center
      spokes = new MechanismLigament2d[NUM_SPOKES];
      for (int i = 0; i < NUM_SPOKES; i++) {
        // Calculate evenly spaced angles for spokes (360° / NUM_SPOKES)
        double spokeAngle = (360.0 / NUM_SPOKES) * i;

        // Create spoke at calculated angle
        spokes[i] =
            root.append(
                new MechanismLigament2d(
                    "Spoke" + i, flywheelRadius, spokeAngle, SPOKE_WIDTH, IDLE_COLOR));
      }
    }

    /**
     * Gets the Mechanism2d object for publishing to SmartDashboard.
     *
     * @return The Mechanism2d visualization
     */
    public Mechanism2d getMechanism() {
      return mech;
    }

    /**
     * Updates the visual representation of the flywheel.
     *
     * <p>Rotates the spokes based on current velocity and changes color based on whether the
     * flywheel is at its target speed.
     *
     * @param velocityRadPerSec The current angular velocity in radians per second
     * @param dt The time step in seconds
     * @param atTarget Whether the flywheel is at its target speed
     */
    public void update(double velocityRadPerSec, double dt, boolean atTarget) {
      // Update visual rotation angle (accumulate over time)
      // Calculate angle change this cycle in radians, then convert to degrees
      double angleChangeRad = velocityRadPerSec * dt;
      visualAngleDeg += Units.radiansToDegrees(angleChangeRad);

      // Keep angle in 0-360 range to prevent overflow
      visualAngleDeg %= 360.0;

      // Determine color based on whether at target speed (green when ready, red when
      // not)
      Color8Bit currentColor = atTarget ? ACTIVE_COLOR : IDLE_COLOR;

      // Update each spoke's angle and color
      for (int i = 0; i < NUM_SPOKES; i++) {
        double spokeAngle = visualAngleDeg + ((360.0 / NUM_SPOKES) * i);
        spokes[i].setAngle(spokeAngle);
        spokes[i].setColor(currentColor);
      }
    }
  }

  public static class HoodMechanism {
    private static final double CANVAS_WIDTH = 400.0;
    private static final double CANVAS_HEIGHT = 400.0;

    private static final double ROOT_X = 200.0;
    private static final double ROOT_Y = 0.5;

    private static final double SPOKE_WIDTH = 2.0;

    private final Mechanism2d m_mech;

    private final MechanismLigament2d[] m_spokes;

    public HoodMechanism(String name, double offset) {
      this.m_mech = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
      MechanismRoot2d root = this.m_mech.getRoot(name + "Root", ROOT_X, ROOT_Y + offset);

      this.m_spokes =
          new MechanismLigament2d[] {
            root.append(
                new MechanismLigament2d(
                    "Direction", 0.2, 20, SPOKE_WIDTH, new Color8Bit(Color.kRed)))
          };
    }

    public Mechanism2d getMechanism() {
      return m_mech;
    }

    public void update(Angle angle) {
      double new_angle = angle.in(Radians);

      new_angle %= 360.0;

      for (int i = 0; i < this.m_spokes.length; i++) {
        double spokeAngle = new_angle + this.m_spokes[i].getAngle();
        this.m_spokes[i].setAngle(spokeAngle);
      }
    }
  }

  /**
   * Helper class for creating and updating an elevator mechanism visualization.
   *
   * <p>This class encapsulates the Mechanism2d visualization for a linear elevator mechanism,
   * including the base, vertical rails, and carriage. It provides a simple interface for updating
   * the elevator's position and color based on its state.
   */
  public static class ElevatorMechanism {
    // ==================== Visualization Constants ====================

    /** Width of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_WIDTH = 200.0;

    /** Height of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_HEIGHT = 400.0;

    /** X position of the mechanism root on the canvas */
    private static final double ROOT_X = 100.0;

    /** Y position of the mechanism root on the canvas */
    private static final double ROOT_Y = 50.0;

    /** Width of the base structure in pixels */
    private static final double BASE_WIDTH = 80.0;

    /** Height of the base structure in pixels */
    private static final double BASE_HEIGHT = 20.0;

    /** Visual width of the carriage in pixels */
    private static final double CARRIAGE_WIDTH = 15.0;

    /** Color of elevator when not at target position */
    private static final Color8Bit MOVING_COLOR = new Color8Bit(Color.kYellow);

    /** Color of elevator when at target position */
    private static final Color8Bit AT_TARGET_COLOR = new Color8Bit(Color.kGreen);

    // ==================== Visualization Components ====================

    /** 2D mechanism visualization */
    private final Mechanism2d mech;

    /** Visual representation of the elevator carriage that updates with simulation */
    private final MechanismLigament2d carriageLigament;

    /** Scale factor to convert inches to pixels */
    private final double inchesToPixels;

    /**
     * Constructs a new ElevatorMechanism visualization.
     *
     * @param name The name to use for the mechanism visualization
     * @param maxHeightInches The maximum height of the elevator in inches
     */
    public ElevatorMechanism(String name, double maxHeightInches) {
      // Calculate scale factor (leave some room at top)
      this.inchesToPixels = (CANVAS_HEIGHT - ROOT_Y - 50.0) / maxHeightInches;

      // Create the 2D mechanism visualization canvas
      mech = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
      MechanismRoot2d root = mech.getRoot(name + "Root", ROOT_X, ROOT_Y);

      // Build the visual hierarchy: Base -> Carriage

      // Base platform (horizontal, dark gray)
      MechanismLigament2d elevatorBase =
          root.append(
              new MechanismLigament2d(
                  "Base", BASE_WIDTH, 0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

      // Vertical rails (for visual reference, gray, extends to max height)
      elevatorBase.append(
          new MechanismLigament2d(
              "Rail", maxHeightInches * inchesToPixels, 90, 3, new Color8Bit(Color.kGray)));

      // The carriage itself (starts yellow, will turn green when at target)
      // Positioned vertically (90 degrees)
      carriageLigament =
          elevatorBase.append(
              new MechanismLigament2d("Carriage", 0, 90, CARRIAGE_WIDTH, MOVING_COLOR));
    }

    /**
     * Gets the Mechanism2d object for publishing to SmartDashboard.
     *
     * @return The Mechanism2d visualization
     */
    public Mechanism2d getMechanism() {
      return mech;
    }

    /**
     * Updates the visual representation of the elevator.
     *
     * @param positionInches The current position of the elevator in inches
     * @param atTarget Whether the elevator is at its target position
     */
    public void update(double positionInches, boolean atTarget) {
      // Convert position to pixels and update carriage height
      double heightPixels = positionInches * inchesToPixels;
      carriageLigament.setLength(heightPixels);

      // Change color based on whether at target position (green = ready, yellow =
      // moving)
      Color8Bit currentColor = atTarget ? AT_TARGET_COLOR : MOVING_COLOR;
      carriageLigament.setColor(currentColor);
    }
  }

  public class IntakeVisualSim {
    // ==================== Visualization Constants ====================

    /** Width of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_WIDTH = 400.0;

    /** Height of the canvas for the mechanism visualization in pixels */
    private static final double CANVAS_HEIGHT = 400.0;

    /** X position of the mechanism root on the canvas */
    private static final double ROOT_X = 200.4;

    /** Y position of the mechanism root on the canvas */
    private static final double ROOT_Y = .2;

    /** Visual width of the arm ligament in pixels */
    private static final double ARM_WIDTH = 8.0;

    /** Number of spokes in the flywheel visualization */
    private static final int NUM_SPOKES = 16;

    /** Width of each spoke in the roller visualization in pixels */
    private static final double SPOKE_WIDTH = 1;

    /** Color of arm when not at target position */
    private static final Color8Bit MOVING_COLOR = new Color8Bit(Color.kYellow);

    /** Color of arm when at target position */
    private static final Color8Bit AT_TARGET_COLOR = new Color8Bit(Color.kGreen);

    /**
     * Angle offset to convert from unit circle coordinates to mechanism ligament coordinates.
     *
     * <p>The arm ligament is attached to a pivot at 90° (on top of vertical tower), but encoder
     * angles follow unit circle convention (0° = right, 90° = up). This offset converts between the
     * two coordinate systems.
     */
    private static final double ANGLE_OFFSET = 0;

    // ==================== Visualization Components ====================

    /** 2D mechanism visualization */
    private final Mechanism2d subsystem;

    /** Visual representation of the arm that updates with simulation */
    private final MechanismLigament2d armLigament;

    /** Visual representation of the flywheel spokes that rotate with simulation */
    private final MechanismLigament2d[] spokes;


    /**
     * Constructs a new ArmMechanism visualization.
     *
     * @param name The name to use for the mechanism visualization
     * @param armVisualLength The length of the arm in pixels
     * @param rollerRadius The radius of the roller in pixels
     */
    public IntakeVisualSim(String name, double armVisualLength, double rollerRadius) {
      //Root
      subsystem = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
      MechanismRoot2d root = subsystem.getRoot(name + "Root", ROOT_X, ROOT_Y);

      // Deplopyment Arm
      armLigament =
          root.append(new MechanismLigament2d("Arm", armVisualLength, 0, ARM_WIDTH, MOVING_COLOR));

      // Rollers
      spokes = new MechanismLigament2d[NUM_SPOKES];
      for (int i = 0; i < NUM_SPOKES; i++) {
        double spokeAngle = (360.0 / NUM_SPOKES) * i;

        // Crea te spoke at calculated angle
        spokes[i] =
            armLigament.append(
                new MechanismLigament2d(
                    "Spoke" + i, rollerRadius, spokeAngle, SPOKE_WIDTH, MOVING_COLOR));
      }
    }

    /**
     * Gets the Mechanism2d object for publishing to SmartDashboard.
     *
     * @return The Mechanism2d visualization
     */
    public Mechanism2d getMechanism() {
      return subsystem;
    }

    /**
     * Updates the visual representation of the arm.
     *
     * @param angleDeg The current angle of the arm in degrees (unit circle convention: 0° = right,
     *     90° = up)
     * @param atTarget Whether the arm is at its target position
     */
    public void updateArm(double angleDeg, boolean atTarget) {
      // Update the visual representation of the arm
      // The arm ligament is relative to the pivot, which sits at 90° absolute (on top
      // of vertical
      // tower)
      // Encoder angle is absolute (0° = right, 90° = up per unit circle)
      // So subtract 90° to convert: encoder 0° → visual -90° (relative to pivot) = 0°
      // absolute
      // (right)
      armLigament.setAngle(angleDeg - ANGLE_OFFSET);

      // Change color based on whether at target position (green = ready, yellow =
      // moving)
      Color8Bit currentColor = atTarget ? AT_TARGET_COLOR : MOVING_COLOR;
      armLigament.setColor(currentColor);
    }

    /**
     * Updates the visual representation of the roller.
     * @param speed The speed of the roller
     */
    public void updateRoller(double speed) {
      // Update each spoke's angle and color
      for (int i = 0; i < NUM_SPOKES; i++) {
        spokes[i].setAngle(spokes[i].getAngle() + speed);
      }
    }
  }
}
