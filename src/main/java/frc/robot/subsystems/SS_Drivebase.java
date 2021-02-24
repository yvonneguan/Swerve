package frc.robot.subsystems;

import javax.annotation.concurrent.GuardedBy;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.MySwerveModule;
import frc.robot.drive.NavX;


public class SS_Drivebase extends SubsystemBase implements UpdateManager.Updatable{

    public static SS_Drivebase instance;

    public static SS_Drivebase getInstance() {
        if (instance == null) {
            instance = new SS_Drivebase();
        }
        return instance;
    }

  public static final double FRONT_LEFT_MODULE_OFFSET = Math.toRadians(0);
  public static final double FRONT_RIGHT_MODULE_OFFSET = Math.toRadians(0);
  public static final double BACK_LEFT_MODULE_OFFSET = Math.toRadians(0);
  public static final double BACK_RIGHT_MODULE_OFFSET = Math.toRadians(0);

  private final Vector2 frontLeftModulePosition = new Vector2(-Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);
  private final Vector2 frontRightModulePosition = new Vector2(-Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);
  private final Vector2 backLeftModulePosition = new Vector2(Constants.TRACKWIDTH / 2.0, Constants.WHEELBASE / 2.0);
  private final Vector2 backRightModulePosition = new Vector2(Constants.TRACKWIDTH / 2.0, -Constants.WHEELBASE / 2.0);

  private final MySwerveModule frontLeftModule = new MySwerveModule(frontLeftModulePosition, FRONT_LEFT_MODULE_OFFSET,
  new TalonFX(Constants.FRONT_LEFT_ANGLEMOTOR_ID), new TalonFX(Constants.FRONT_LEFT_DRIVEMOTOR_ID), new CANCoder(Constants.FRONT_LEFT_CANCODER_ID));

  private final MySwerveModule frontRightModule = new MySwerveModule(frontRightModulePosition, FRONT_RIGHT_MODULE_OFFSET,
  new TalonFX(Constants.FRONT_RIGHT_ANGLEMOTOR_ID), new TalonFX(Constants.FRONT_RIGHT_DRIVEMOTOR_ID), new CANCoder(Constants.FRONT_RIGHT_CANCODER_ID));

  private final MySwerveModule backLeftModule = new MySwerveModule(backLeftModulePosition, BACK_LEFT_MODULE_OFFSET,
  new TalonFX(Constants.BACK_LEFT_ANGLEMOTOR_ID), new TalonFX(Constants.BACK_LEFT_DRIVEMOTOR_ID), new CANCoder(Constants.BACK_LEFT_CANCODER_ID));

  private final MySwerveModule backRightModule = new MySwerveModule(backRightModulePosition, BACK_RIGHT_MODULE_OFFSET,
  new TalonFX(Constants.BACK_RIGHT_ANGLEMOTOR_ID), new TalonFX(Constants.BACK_RIGHT_DRIVEMOTOR_ID), new CANCoder(Constants.BACK_RIGHT_CANCODER_ID));

  private final MySwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

  private final SwerveKinematics kinematics = new SwerveKinematics(frontLeftModulePosition,frontRightModulePosition,
  backLeftModulePosition, backRightModulePosition);
  
  private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

  private final Object sensorLock = new Object();
  @GuardedBy("sensorLock")
  private final NavX navX = new NavX(Port.kMXP, Constants.NAVX_UPDATE_RATE);

  private final Object kinematicsLock = new Object();
  @GuardedBy("kinematicsLock")
  private RigidTransform2 pose = RigidTransform2.ZERO;

  private final Object stateLock = new Object();
  @GuardedBy("stateLock")
  private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);

  private NetworkTableEntry poseXEntry;
  private NetworkTableEntry poseYEntry;
  private NetworkTableEntry poseAngleEntry;

  private NetworkTableEntry fieldOrientedEntry;  
  private NetworkTableEntry gyroAngleEntry;
  private NetworkTableEntry correctionAngleEntry;

  private NetworkTableEntry driveSignalXEntry;
  private NetworkTableEntry driveSignalYEntry;
  private NetworkTableEntry driveSignalRotationEntry;

  private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];
  private NetworkTableEntry[] moduleVelocityEntries = new NetworkTableEntry[modules.length];

  public SS_Drivebase() {

    /*
    ShuffleboardTab drivebaseTab = Shuffleboard.getTab("Drivebase");
    poseXEntry = drivebaseTab.add("Pose X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    poseYEntry = drivebaseTab.add("Pose Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
    poseAngleEntry = drivebaseTab.add("Pose Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();

    ShuffleboardLayout driveSignalContainer = drivebaseTab.getLayout("Drive Signal", BuiltInLayouts.kGrid)
        .withPosition(0, 3).withSize(3, 1);
    driveSignalYEntry = driveSignalContainer.add("Drive Signal Strafe", 0.0).getEntry();
    driveSignalXEntry = driveSignalContainer.add("Drive Signal Forward", 0.0).getEntry();
    driveSignalRotationEntry = driveSignalContainer.add("Drive Signal Rotation", 0.0).getEntry();

    ShuffleboardLayout frontLeftModuleContainer = drivebaseTab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withPosition(5, 0).withSize(2, 2);
    moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();
    moduleVelocityEntries[0] = frontLeftModuleContainer.add("Velocity", 0.0).getEntry();

    ShuffleboardLayout frontRightModuleContainer = drivebaseTab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withPosition(7, 0).withSize(2, 2);
    moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();
    moduleVelocityEntries[1] = frontRightModuleContainer.add("Velocity", 0.0).getEntry();

    ShuffleboardLayout backLeftModuleContainer = drivebaseTab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withPosition(5, 2).withSize(2, 2);
    moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();
    moduleVelocityEntries[2] = backLeftModuleContainer.add("Velocity", 0.0).getEntry();

    ShuffleboardLayout backRightModuleContainer = drivebaseTab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withPosition(7, 2).withSize(2, 2);
    moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();
    moduleVelocityEntries[3] = backRightModuleContainer.add("Velocity", 0.0).getEntry();

    ShuffleboardLayout fieldOrientedContainer = drivebaseTab.getLayout("Field Oriented", BuiltInLayouts.kList)
        .withPosition(1, 0).withSize(1, 1);
    fieldOrientedEntry = fieldOrientedContainer.add("Field Oriented", 0.0).getEntry();

    ShuffleboardLayout gyroContainer = drivebaseTab.getLayout("Gyro", BuiltInLayouts.kList).withPosition(1, 1).withSize(1, 1);
    gyroAngleEntry = gyroContainer.add("Gyro Angle", 0.0).getEntry();

    ShuffleboardLayout correctionContainer = drivebaseTab.getLayout("Correction", BuiltInLayouts.kList).withPosition(1, 2).withSize(1, 1);
    correctionAngleEntry = correctionContainer.add("Correction", 0.0).getEntry();
    */
  }

  public RigidTransform2 getPose() {
      synchronized (kinematicsLock) {
          return pose;
      }
  }

  public void resetGyroAngle(Rotation2 angle) {
      synchronized (sensorLock) {
          navX.setAdjustmentAngle(navX.getUnadjustedAngle().rotateBy(angle.inverse()));
      }
  }

  public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
      synchronized (stateLock) {
          driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, fieldOriented);
      }
  }

  public void drive(HolonomicDriveSignal driveSignal) {
      synchronized(stateLock) {
          this.driveSignal = driveSignal;
      }
  }

  public void resetPose() {
      synchronized(kinematicsLock) {
          pose = new RigidTransform2(Vector2.ZERO, Rotation2.ZERO);
      }
  }

  public void resetPoseTranslation() {
      synchronized(kinematicsLock) {
          RigidTransform2 previousPose = pose;
          odometry.resetPose(new RigidTransform2(Vector2.ZERO, previousPose.rotation));
      }
  }

  public MySwerveModule[] getModules() {
      return modules;
  }

  @Override
  public void update(double timestamp, double dt) {
      updateOdometry(dt);

      HolonomicDriveSignal driveSignal;
      synchronized (stateLock) {
          driveSignal = this.driveSignal;
      }

      updateModules(driveSignal, dt);
  }

  private void updateOdometry(double dt) {
      Vector2[] moduleVelocities = new Vector2[modules.length];
      for (int i = 0; i < modules.length; i++) {
          var module = modules[i];
          module.updateSensors();

          moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.readAngle())).scale(module.getCurrentVelocity());
      }

      Rotation2 angle;
      synchronized (sensorLock) {
          angle = navX.getAngle();
      }

      RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

      synchronized (kinematicsLock) {
          this.pose = pose;
      }
  }

  private void updateModules(HolonomicDriveSignal signal, double dt) {
      RigidTransform2 pose = getPose();
      ChassisVelocity velocity;
    
      if (signal == null) {
          velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
      } else if (signal.isFieldOriented()) {

          Rotation2 correction = pose.rotation;
          velocity = new ChassisVelocity(signal.getTranslation().rotateBy(correction),signal.getRotation());
          correctionAngleEntry.setDouble(correction.toRadians());

      } else {
          velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
      }

      Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
      SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

      for (int i = 0; i < modules.length; i++) {
        var module = modules[i];
        module.setTargetVelocity(moduleOutputs[i]);
        module.updateState(dt);
      }
      poseXEntry.setDouble(pose.translation.x);
      poseYEntry.setDouble(pose.translation.y);
      poseAngleEntry.setDouble(pose.rotation.toDegrees());

      fieldOrientedEntry.setDouble( signal == null ? 0 : (signal.isFieldOriented() ? 1 : 0));   
      gyroAngleEntry.setDouble(navX.getAngle().toDegrees());
  }

  @Override
  public void periodic() {
      for (int i = 0; i < modules.length; i++) {
          var module = modules[i];
          moduleAngleEntries[i].setDouble(Math.toDegrees(module.readAngle()));
          moduleVelocityEntries[i].setDouble(module.getCurrentVelocity());
      }
      synchronized(stateLock) {
          driveSignalYEntry.setDouble(driveSignal.getTranslation().y);
          driveSignalXEntry.setDouble(driveSignal.getTranslation().x);
          driveSignalRotationEntry.setDouble(driveSignal.getRotation());
      }
  }
}