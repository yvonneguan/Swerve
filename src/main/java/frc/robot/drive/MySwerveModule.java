package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

public final class MySwerveModule extends SwerveModule {

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private TalonFXSensorCollection integratedSensorAngle;
    private TalonFXSensorCollection integratedSensorDrive;
    private CANCoder angleAbsoluteEncoder;
    
	private double offsetAngle;
    private double wheelRevolutionsPerUnit;

    public MySwerveModule(Vector2 modulePosition, double offsetAngle, TalonFX driveMotor, TalonFX angleMotor, CANCoder angleAbsoluteEncoder) {
        super(modulePosition);
        this.offsetAngle = offsetAngle;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleAbsoluteEncoder = angleAbsoluteEncoder;

        integratedSensorAngle = new TalonFXSensorCollection(angleMotor);
        integratedSensorDrive = new TalonFXSensorCollection(driveMotor);
        
        angleMotor.config_kP(0, 0);
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0);

        driveMotor.config_kP(0, 0);
        driveMotor.config_kI(0, 0);
        driveMotor.config_kD(0, 0);
    }

    public void setDriveTicksPerUnit(double driveTicksPerUnit) {
        wheelRevolutionsPerUnit = driveTicksPerUnit;
    }

    @Override
    public double readAngle() {
        angleAbsoluteEncoder.configMagnetOffset(Math.toDegrees(offsetAngle));
        double angle = Math.toRadians(angleAbsoluteEncoder.getPosition());
        angle %= 2 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public void setAngle() {
        integratedSensorAngle.setIntegratedSensorPosition(readAngle() * 2048 * (2.0 * Math.PI), 0);
    }

    @Override
    public double getCurrentVelocity() {
        return integratedSensorDrive.getIntegratedSensorVelocity();
    }

    @Override
    public void setDriveOutput(double output) {
        driveMotor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public double readDistance() {
        return integratedSensorDrive.getIntegratedSensorPosition() / wheelRevolutionsPerUnit;
    }

    @Override
	public void setTargetAngle(double angle) {
        angleMotor.set(ControlMode.Position, angle);
    }
}