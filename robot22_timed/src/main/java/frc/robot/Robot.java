
package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Robot extends TimedRobot {

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

      public static boolean objectInCanStart = false;
      public static boolean goBackCanStart = false;
      public static boolean objectShootCanStart = false;

    }

  }

  public static final class Subsystems {

    public static final class Drive {

      private final WPI_TalonSRX DriveLeftTalonSRX1 = new WPI_TalonSRX(Constants.CANPortConstants.leftFrontTalon);
      private final WPI_TalonSRX DriveLeftTalonSRX2 = new WPI_TalonSRX(Constants.CANPortConstants.leftBackTalon);
      private final WPI_VictorSPX DriveRightVictorSPX1 = new WPI_VictorSPX(Constants.CANPortConstants.rightFrontVictor);
      private final WPI_VictorSPX DriveRightVictorSPX2 = new WPI_VictorSPX(Constants.CANPortConstants.rightBackVictor);
      private final MotorControllerGroup DriveLeftGroup = new MotorControllerGroup(DriveLeftTalonSRX1,
          DriveLeftTalonSRX2);
      private final MotorControllerGroup DriveRightGroup = new MotorControllerGroup(DriveRightVictorSPX1,
          DriveRightVictorSPX2);
      private final DifferentialDrive difDrive = new DifferentialDrive(DriveLeftGroup, DriveRightGroup);

      private NeutralMode defaultMode = NeutralMode.Brake;

      public void tankDrive(double leftSpeed, double rightSpeed) {
        difDrive.tankDrive(leftSpeed, rightSpeed);
      }

      public void stopMotors() {
        DriveLeftGroup.stopMotor();
        DriveRightGroup.stopMotor();
      }
    }

    public static final class Hopper {

      private final WPI_VictorSPX hopperRedline = new WPI_VictorSPX(Constants.CANPortConstants.kHopperVictor);

      public void openHopper() {
        hopperRedline.set(Constants.HopperConstants.kHopperSpeed);
      }

      public void reverseHopper() {
        hopperRedline.set(-Constants.HopperConstants.kHopperSpeed);
      }

      public void stopHopper() {
        hopperRedline.set(0);
      }

    }

    public static final class Indexer {
      private final WPI_VictorSPX IndexerRedline = new WPI_VictorSPX(Constants.CANPortConstants.kIndexerVictor);

      public void openIndexer() {
        IndexerRedline.set(Constants.IndexerConstants.kIndexerSpeed);
      }

      public void reverseIndexer() {
        IndexerRedline.set(-Constants.IndexerConstants.kIndexerSpeed);
      }

      public void stopIndexer() {
        IndexerRedline.set(0);
      }
    }

    public static final class Intake {

      private final WPI_VictorSPX intakeRedline = new WPI_VictorSPX(Constants.CANPortConstants.kIntakeVictor);
      private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, Constants.PCM.kSolenoidChannel1, Constants.PCM.kSolenoidChannel2);

      public void openIntake() {
        intakeRedline.set(Constants.IntakeConstants.kIntakeSpeed);
      }

      public void reverseIntake() {
        intakeRedline.set(-Constants.IntakeConstants.kIntakeSpeed);
      }

      public void stopIntake() {
        intakeRedline.set(0);
      }

      public void openIntakeSolenoid() {
        intakeSolenoid.set(Value.kForward);
      }

      public void reverseIntakeSolenoid() {
        intakeSolenoid.set(Value.kReverse);
      }

      public void stopIntakeSolenoid() {
        intakeSolenoid.set(Value.kOff);
      }
    }

    public static final class LimitBlock {

      private final WPI_VictorSPX LimitBlockRedline = new WPI_VictorSPX(Constants.CANPortConstants.kLimitBVictor);

      public void openLimitBlock() {
        LimitBlockRedline.set(Constants.LimitBConstants.kLimitBSpeed);
      }

      public void reverseLimitBlock() {
        LimitBlockRedline.set(-Constants.LimitBConstants.kLimitBSpeed);
      }

      public void stopLimitBlock() {
        LimitBlockRedline.set(0);
      }
    }

    public static final class Shooter {

      private final WPI_VictorSPX ShooterRedlineLeft = new WPI_VictorSPX(Constants.CANPortConstants.kShooterVictorLeft);
      private final WPI_VictorSPX ShooterRedlineRight = new WPI_VictorSPX(
          Constants.CANPortConstants.kShooterVictorRight);
      private final MotorControllerGroup ShooterRedlines = new MotorControllerGroup(ShooterRedlineLeft,
          ShooterRedlineRight);

      private final Timer ShooterTimer = new Timer();

      public void openShooter() {
        ShooterRedlines.set(Constants.ShooterConstants.kShooterSpeed);
      }

      public void stopShooter() {
        ShooterRedlines.set(0);
      }

      public void setShooter(double speed) {
        ShooterRedlines.set(speed);
      }

      public void resetStartTimer() {
        ShooterTimer.reset();
        ShooterTimer.start();
      }

      public double getTimer() {
        return ShooterTimer.get();
      }

      public void stopTimer() {
        ShooterTimer.stop();
      }

    }

    public static final class Turret {

      private final WPI_VictorSPX TurretRedline = new WPI_VictorSPX(Constants.CANPortConstants.kTurretVictor);

      public void stopTurret() {
        TurretRedline.set(0);
      }

      public void setTurret(double speed) {
        TurretRedline.set(speed);
      }
    }

  }

  private final Subsystems.Drive m_drive = new Subsystems.Drive();
  private final Subsystems.Hopper m_hopper = new Subsystems.Hopper();
  private final Subsystems.Indexer m_indexer = new Subsystems.Indexer();
  private final Subsystems.Intake m_intake = new Subsystems.Intake();
  private final Subsystems.LimitBlock m_limitBlock = new Subsystems.LimitBlock();
  private final Subsystems.Shooter m_shooter = new Subsystems.Shooter();
  private final Subsystems.Turret m_turret = new Subsystems.Turret();

  public static final Timer autonomous_timer = new Timer();

  public Joystick driverController = new Joystick(Constants.JoystickConstants.kDriverControllerPort);

  public JoystickButton objectInButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kObjectInComButton);
  public JoystickButton objectOutButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kObjectOutComButton);
  public JoystickButton objectShootButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kObjectShootComButton);
  public JoystickButton solenoidOnButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kSolenoidOnButton);
  public JoystickButton solenoidReverseButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kSolenoidRevButton);
  public JoystickButton TurretTurnLeftButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kTurretTurnLeftButton);
  public JoystickButton TurretTurnRightButton = new JoystickButton(driverController,
      Constants.JoystickConstants.kTurretTurnRightButton);

  public boolean isTurretTurnedLeft = true;
  public boolean isSolenoidOn = false;
  public boolean isReadyToShoot = false;

  // COMMANDS

  public void ObjectOutCommand() {
    m_intake.reverseIntake();
    m_hopper.reverseHopper();
    m_indexer.reverseIndexer();
    m_limitBlock.reverseLimitBlock();
  }

  public void ObjectInCommand() {
    m_intake.openIntake();
    m_hopper.openHopper();
    m_indexer.openIndexer();

  }

  public void ObjectShootCommand() {
    m_shooter.resetStartTimer();

    m_limitBlock.openLimitBlock();

    if (m_shooter.getTimer() >= Constants.ShooterConstants.kWaitToStartSecs) {
      m_shooter.openShooter();
    }

  }

  @Override
  public void robotInit() {

    m_drive.DriveLeftTalonSRX1.setInverted(Constants.DriveConstants.kSetLeft1Inverted);
    m_drive.DriveLeftTalonSRX2.setInverted(Constants.DriveConstants.kSetLeft2Inverted);
    m_drive.DriveRightVictorSPX1.setInverted(Constants.DriveConstants.kSetRight1Inverted);
    m_drive.DriveRightVictorSPX2.setInverted(Constants.DriveConstants.kSetRight2Inverted);
    m_drive.DriveLeftTalonSRX1.setNeutralMode(m_drive.defaultMode);
    m_drive.DriveLeftTalonSRX2.setNeutralMode(m_drive.defaultMode);
    m_drive.DriveRightVictorSPX1.setNeutralMode(m_drive.defaultMode);
    m_drive.DriveRightVictorSPX2.setNeutralMode(m_drive.defaultMode);

    m_hopper.hopperRedline.setInverted(Constants.HopperConstants.kSetMotorInverted);

    m_indexer.IndexerRedline.setInverted(Constants.IndexerConstants.kSetMotorInverted);

    m_limitBlock.LimitBlockRedline.setInverted(Constants.LimitBConstants.kSetMotorInverted);

    m_shooter.ShooterRedlineLeft.setInverted(Constants.ShooterConstants.kSetLeftMotorInverted);
    m_shooter.ShooterRedlineRight.setInverted(Constants.ShooterConstants.kSetRightMotorInverted);

    m_turret.TurretRedline.setInverted(Constants.TurretConstants.kSetMotorInverted);
  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {

    autonomous_timer.reset();
    autonomous_timer.start();

  }

  @Override
  public void autonomousPeriodic() {

    if (autonomous_timer.get() <= Constants.AutoConstants.kMoveInSeconds) {
      m_drive.tankDrive(Constants.AutoConstants.kMoveSpeed, Constants.AutoConstants.kMoveSpeed);
    }

    else if (autonomous_timer.get() > Constants.AutoConstants.kMoveInSeconds) {
      m_drive.stopMotors();
      autonomous_timer.stop();
      autonomous_timer.reset();
      Constants.AutoConstants.objectInCanStart = true;
    }

    if (Constants.AutoConstants.objectInCanStart) {
      autonomous_timer.start();

      if (autonomous_timer.get() <= Constants.AutoConstants.kTakeObjectInSeconds) {
        m_intake.openIntake();
        m_hopper.openHopper();
        m_indexer.openIndexer();
      }

      else if (autonomous_timer.get() > Constants.AutoConstants.kTakeObjectInSeconds) {
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_indexer.stopIndexer();
        autonomous_timer.stop();
        autonomous_timer.reset();
        Constants.AutoConstants.objectInCanStart = false;
        Constants.AutoConstants.goBackCanStart = true;
      }
    }

    if (Constants.AutoConstants.goBackCanStart) {
      autonomous_timer.start();
      if (autonomous_timer.get() <= Constants.AutoConstants.kMoveBackInSeconds) {
        m_drive.tankDrive(Constants.AutoConstants.kMoveBackSpeed, Constants.AutoConstants.kMoveBackSpeed);
      } else if (autonomous_timer.get() > Constants.AutoConstants.kMoveBackInSeconds) {
        m_drive.stopMotors();
        autonomous_timer.stop();
        autonomous_timer.reset();
        Constants.AutoConstants.goBackCanStart = false;
        Constants.AutoConstants.objectShootCanStart = true;
      }
    }

    if (Constants.AutoConstants.objectShootCanStart) {
      autonomous_timer.start();
      if (autonomous_timer.get() <= Constants.AutoConstants.kShootObjectInSeconds) {
        m_limitBlock.openLimitBlock();
        m_shooter.setShooter(Constants.AutoConstants.kShootingSpeed);

      } else if (autonomous_timer.get() > Constants.AutoConstants.kShootObjectInSeconds) {
        m_limitBlock.stopLimitBlock();
        m_shooter.stopShooter();
        m_indexer.stopIndexer();
        autonomous_timer.stop();
        autonomous_timer.reset();
        Constants.AutoConstants.objectShootCanStart = false;
      }
    }

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    m_drive.tankDrive(driverController.getRawAxis(Constants.JoystickConstants.kDriveLeftAxis),
        driverController.getRawAxis(Constants.JoystickConstants.kDriveRightAxis));

    if (objectInButton.get()) {
      ObjectInCommand();
    }

    else if (objectOutButton.get()) {
      ObjectOutCommand();
    }

    else if (objectShootButton.get()) {
      ObjectShootCommand();
    }

    else if (solenoidOnButton.get()) {

      if (!isSolenoidOn) {
        m_intake.openIntakeSolenoid();
        isSolenoidOn = true;
      } else {

      }

    }

    else if (solenoidReverseButton.get()) {

      if (isSolenoidOn) {
        m_intake.reverseIntakeSolenoid();
        isSolenoidOn = false;
      } else {

      }

    }

    else if (TurretTurnLeftButton.get()) {
      m_turret.setTurret(Constants.TurretConstants.kTurretSpeed);
    }

    else if (TurretTurnRightButton.get()) {
      m_turret.setTurret(-Constants.TurretConstants.kTurretSpeed);
    }

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