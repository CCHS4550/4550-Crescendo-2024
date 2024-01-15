package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Class for controlling a swerve module. Each module has 2 motors, one for
 * driving and one for turning, as well as an absolute encoder.
 * 
 * <p>
 * Swerve modules are set to different positions and vectors in the
 * SwerveModuleState format.
 * SwerveModuleState takes in a drive speed in meters per second and an angle in
 * radians in the
 * format of Rotation2d.
 */
public class SwerveModule extends SubsystemBase {
    private CCSparkMax driveMotor;
    private CCSparkMax turnMotor;

    private PIDController turningPIDController, drivingPidController;
    private SimpleMotorFeedforward driveFeedforward;

    // private SparkPIDController turningPIDController;

    private AnalogEncoder absoluteEncoder;
    private double absoluteEncoderOffset;
    private String name;

    // adjust absoluteEncoderChannel to possibly be absoluteEncoderAnalogInput
    /**
     * Creates a SwerveModule object with a defined drive motor, turn motor, and
     * absolute encoder.
     * 
     * @param driveMotor             The drive motor in CCSparkMax format.
     * @param turnMotor              The turn motor in CCSparkMax format.
     * @param absoluteEncoderChannel The port of the absolute encoder.
     * @param absoluteEncoderOffset  The offset of the absolute encoder in radians.
     */
    public SwerveModule(CCSparkMax driveMotor, CCSparkMax turnMotor, int absoluteEncoderChannel,
            double absoluteEncoderOffset, String name) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.absoluteEncoder = new AnalogEncoder(absoluteEncoderChannel);

        this.absoluteEncoder.setDistancePerRotation(2 * Math.PI);

        // this.absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        // turningPIDController = new SparkPIDController(.5, 0, 0);
        // turningPIDController = new SparkPIDController();
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        drivingPidController = new PIDController(0.5, 0, 0);

        driveFeedforward = new SimpleMotorFeedforward(RobotMap.DRIVEKS, RobotMap.DRIVEKV);

        this.name = name;
        resetEncoders();
    }

    /**
     * Gets the encoder value of the drive motor in meters.
     * 
     * @return The encoder value of the drive motor.
     */
    public double getDrivePosition() {
        return driveMotor.getPosition(); // should be in meters?
    }

    /**
     * Gets the encoder value of the turn motor.
     * 
     * @return The encoder value of the turn motor.
     */
    public double getTurnPosition() {
        return turnMotor.getPosition(); // should be in radians?
    }

    /**
     * Gets the speed of the drive motor. Obtained using max speed given 12V of power.
     * 
     * @return The speed of the drive motor in m/s.
     */
    public double getDriveVelocity() {
        return RobotMap.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL / 12 * driveMotor.getBusVoltage();
    }

    /**
     * Gets the speed of the turn motor.
     * 
     * @return The speed of the turn motor between -1 and 1.
     */
    public double getTurnVelocity() {
        return turnMotor.getSpeed();
    }

    /**
     * Gets the voltage being supplied to the turn motor.
     * 
     * @return The voltage being supplied to the turn motor.
     */
    public double getDriveVoltage() {
        return driveMotor.get() * driveMotor.getBusVoltage();
    }

    /**
     * Gets the voltage being supplied to the turn motor.
     * 
     * @return The voltage being supplied to the turn motor.
     */
    public double getTurnVoltage() {
        return turnMotor.get() * turnMotor.getBusVoltage();
    }

    /**
     * Gets the reading of the absolute encoder with offset.
     * 
     * @return The value of the absolute encoder in radians with the offset applied.
     */
    public double getAbsoluteEncoderRadiansOffset() {
        return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition()) - absoluteEncoderOffset;
    }


    /**
     * Gets the reading of the absolute encoder with offset.
     * Used for getting the offset. 
     * 
     * @return The value of the absolute encoder in radians without the offset applied.
     */
    public double getAbsoluteEncoderRadiansNoOffset() {
        return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition());
    }

    /**
     * Resets the drive and turn motor encoders. The drive motor is set to
     * 0 while the turn motor is set to the value of the absolute encoder.
     */
    public void resetEncoders() {
        driveMotor.reset();
        turnMotor.setPosition(getAbsoluteEncoderRadiansOffset());
        // turnMotor.reset();
    }

    /**
     * Gets the state of the module.
     * 
     * @return The state of the swerve module in SwerveModuleState format.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRadiansOffset()));
    }

    /**
     * Sets the state of the module.
     * @param state The state to set the swerve module to in SwerveModuleState format.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        if(Math.abs(desiredState.speedMetersPerSecond) <= .005){
            stop();
            return;
        }

        Rotation2d encoderRotation = new Rotation2d(getState().angle.getRadians());

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
        // Minimizes side drift when driving
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        //integrate max speed here
            // isOpenLoop would be true in teleop perhaps because some drivers, like ours prefers it that way

        // NEED TO DO FEED FORWARD STUFF
        driveMotor.setVoltageFromSpeed(desiredState.speedMetersPerSecond);
        turnMotor.setVoltageFromSpeed(turningPIDController.calculate(getAbsoluteEncoderRadiansOffset(), desiredState.angle.getRadians()));
    }

    /**
     * Sets the speed of the drive and turn motors to 0.
     */
    public void stop() {
        driveMotor.setVoltageFromSpeed(0);
        turnMotor.setVoltageFromSpeed(0);
    }

    /**
     * Method for testing purposes
     * 
     * @param driveSpeed Speed of the drive motor.
     * @param turnSpeed  Speed of the turn motor.
     */
    public void driveAndTurn(double driveSpeed, double turnSpeed) {
        driveMotor.setVoltageFromSpeed(driveSpeed);
        turnMotor.setVoltageFromSpeed(turnSpeed);
    }

    public void printEncoders() {
        System.out.println(name + "\nDrive Encoder: " + driveMotor.getPosition() + "\nTurn Encoder: "
                + turnMotor.getPosition() + "\n");
    }

    public void resetAbsoluteEncoder() {
        absoluteEncoder.reset();
    }

    public void printAbsoluteEncoder() {
        System.out.println(name + ": " + absoluteEncoder.getDistance());
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}
