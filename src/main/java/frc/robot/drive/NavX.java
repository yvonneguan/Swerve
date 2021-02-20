package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.*;

/**
 * Add your docs here.
 */
public class NavX extends Gyroscope{
    private AHRS navX;

    /** 
     * @param updateRate should always be set to the max of 200
    */
    public NavX(I2C.Port i2cport, byte updateRate) {
        navX = new AHRS(i2cport, updateRate);
    }
    /** 
     * @param updateRate should always be set to the max of 200
    */
    public NavX(SPI.Port spiport, byte updateRate) {
        navX = new AHRS(spiport, updateRate);
    }
    /** 
     * @param updateRate should always be set to the max of 200
    */
    public NavX(SerialPort.Port port, byte updateRate) {
        navX = new AHRS(port, SerialDataType.kProcessedData, updateRate);
    }

    
    @Override
    public void calibrate() {
        navX.reset();
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    @Override
    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }

}