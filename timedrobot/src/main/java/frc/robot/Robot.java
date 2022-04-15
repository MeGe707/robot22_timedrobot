
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Constants.CANPortConstants;
import frc.robot.Robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {

  // CONSTANTS
  public static final class Constants {

    /*
     * driver gamepad
     * button a (no 1) - take the ball out
     * button b (no 2) - close intake solenoid
     * button x (no 3) - open intake solenoid
     * button y (no 4) - take the ball in
     * joystick left (axis 1) - tank drive left
     * joystick right (axis 5) - tank drive right
     * operator gamepad
     * button y (no ?) - shoot the ball
     * left front top button (no ?) - turret to left
     * right front top button (no ?) - turret to right
     */

    public static final class JoystickConstants {
      // driver controller
      public static final int kDriverControllerPort = 0;
      public static final int kDriveLeftAxis = 1;
      public static final int kDriveRightAxis = 5;
      public static final int kObjectInComButton = 4;
      public static final int kObjectOutComButton = 1;
      public static final int kSolenoidOnButton = 3;
      public static final int kSolenoidRevButton = 2;
      // operator controller
      public static final int kOprtControllerPort = 1;
      public static final int kObjectShootComButton = 1;
      public static final int kTurretTurnLeftButton = 2;
      public static final int kTurretTurnRightButton = 3;
    }

    public static final class CANPortConstants {
      public static final int leftBackTalon = 1;
      public static final int rightBackVictor = 2;
      public static final int leftFrontTalon = 3;
      public static final int rightFrontVictor = 4;
      public static final int kHopperVictor = 5;
      public static final int kIndexerVictor = 6;
      public static final int kIntakeVictor = 7;
      public static final int kLimitBVictor = 8;
      public static final int kShooterVictorLeft = 9;
      public static final int kShooterVictorRight = 10;
      public static final int kTurretVictor = 11;
    }

    public static final class PCM {
      public static final int kSolenoidChannel1 = 0;
      public static final int kSolenoidChannel2 = 1;
    }

    public static final class HopperConstants {
      public static final double kHopperSpeed = 0.7;
      public static final boolean kSetMotorInverted = false;
    }

    public static final class IndexerConstants {
      public static final double kIndexerSpeed = 0.7;
      public static final boolean kSetMotorInverted = false;
    }

    public static final class IntakeConstants {
      public static final double kIntakeSpeed = 0.7;
      public static final boolean kSetMotorInverted = true;

    }

    public static final class LimitBConstants {
      public static final double kLimitBSpeed = 0.95;
      public static final boolean kSetMotorInverted = false;
    }

    public static final class ShooterConstants {
      public static final double kShooterSpeed = 0.95;
      public static final double kWaitToStartSecs = 1;
      public static final boolean kSetLeftMotorInverted = false;
      public static final boolean kSetRightMotorInverted = true;
    }

    public static final class TurretConstants {
      public static final double kTurretSpeed = 0.6;
      public static final boolean kSetMotorInverted = false;
    }

    public static final class DriveConstants {
      // drive in half speed mode
      public static final double kSlowModeSpeed = 0.5;
      public static final boolean kSetLeft1Inverted = true;
      public static final boolean kSetLeft2Inverted = true;
      public static final boolean kSetRight1Inverted = false;
      public static final boolean kSetRight2Inverted = false;
    }

    public static final class AutoConstants {
      public static final double kMoveInSeconds = 3;
      public static final double kMoveSpeed = 0.3;
      public static final double kTakeObjectInSeconds = 2;
      public static final double kMoveBackInSeconds = 3;
      public static final double kMoveBackSpeed = -0.3;
      public static final double kShootObjectInSeconds = 4;
      public static final double kShootingSpeed = 0.7;
    }

  }
  // CONSTANTS

  private final WPI_TalonSRX leftTalonSRX1 = new WPI_TalonSRX(CANPortConstants.leftFrontTalon);
  private final WPI_TalonSRX leftTalonSRX2 = new WPI_TalonSRX(CANPortConstants.leftBackTalon);
  private final WPI_VictorSPX rightVictorSPX1 = new WPI_VictorSPX(CANPortConstants.rightFrontVictor);
  private final WPI_VictorSPX rightVictorSPX2 = new WPI_VictorSPX(CANPortConstants.rightBackVictor);
  private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftTalonSRX1, leftTalonSRX2);
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightVictorSPX1, rightVictorSPX2);
  private final DifferentialDrive difDrive = new DifferentialDrive(leftGroup, rightGroup);

  private NeutralMode defaultMode = NeutralMode.Brake;

  public void DriveSubsystem() {
    leftTalonSRX1.setInverted(DriveConstants.kSetLeft1Inverted);
    leftTalonSRX2.setInverted(DriveConstants.kSetLeft2Inverted);
    rightVictorSPX1.setInverted(DriveConstants.kSetRight1Inverted);
    rightVictorSPX2.setInverted(DriveConstants.kSetRight2Inverted);
    leftTalonSRX1.setNeutralMode(defaultMode);
    leftTalonSRX2.setNeutralMode(defaultMode);
    rightVictorSPX1.setNeutralMode(defaultMode);
    rightVictorSPX2.setNeutralMode(defaultMode);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    difDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stopMotors() {
    leftGroup.stopMotor();
    rightGroup.stopMotor();
  }

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
