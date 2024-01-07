package frc.helpers;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.*;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

//Documention: https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html 

public class CCSparkMax extends CANSparkMax{

    private String name;
    private String shortName;
    public  SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private double voltageConversionFactor;

    /**
     * CCSparkMax allows us to easily control Spark Max motor controllers
     * Information on modes can be found in the Spark Max documentation
     * @param deviceID The CAN channel of the motor controller
     * @param controlMode Specify whether the motor controller is operating in Brushed or Brushless mode
     * @param idleMode Specify whether the motor controller is set to Coast or Brake mode
     * @param reverse Reverses the direction of the motor controller
     * @param encoder If the motor has an encoder or not
     * @param positionConversionFactor Conversion rate for position from rotations to desired unit
     * @param velocityConversionFactor Conversion rate for velocity from rotations per minute to desired unit
     */
    public CCSparkMax(String name, String shortName, int deviceID, MotorType controlMode, IdleMode idleMode,
     boolean reverse, double positionConversionFactor, double velocityConversionFactor){
        super(deviceID, controlMode);
        this.name = name;
        this.shortName = shortName;
        
        super.setInverted(reverse);
        

        pidController = super.getPIDController();
        this.encoder = super.getEncoder();
        this.setPositionConversionFactor(positionConversionFactor);
        this.setVelocityConversionFactor(velocityConversionFactor);
    }
    public CCSparkMax(String name, String shortName, int deviceID, MotorType controlMode, IdleMode idleMode,
     boolean reverse){
        super(deviceID, controlMode);
        this.name = name;
        this.shortName = shortName;
        
        super.setInverted(reverse);
        

        pidController = super.getPIDController();
        this.encoder = super.getEncoder();
        this.setPositionConversionFactor(1);
        this.setVelocityConversionFactor(1);
    }
    public CCSparkMax(String name, String shortName, int deviceID, MotorType controlMode, IdleMode idleMode,
     boolean reverse, double encoder){
        super(deviceID, controlMode);
        this.name = name;
        this.shortName = shortName;
        
        super.setInverted(reverse);
        
        if(encoder < 0) return;
        pidController = super.getPIDController();
        this.encoder = super.getEncoder();
        this.setPositionConversionFactor(1);
        this.setVelocityConversionFactor(1);
    }

   

    public void reset(){
        encoder.setPosition(0);
    }

    public void setReferencePosition(double pos){
        pidController.setReference(pos, ControlType.kPosition);
    }

    /**
     * Sets the speed of the motor controller
     * @param speed The speed that will be set (-1.0 to 1.0)
     */
    public void set(double speed) {
        super.set(speed);
    }
    
    public void setVoltage(double volts) {
        super.setVoltage(volts);
    }
    
    public void setVoltageFromSpeed(double speed) {
        super.setVoltage(speed * voltageConversionFactor);
    }

    public void disable(){
        super.disable();
    }

    public double getSpeed(){
        return super.get();
    }

    /**
     * Sets the Position Conversion Factor for the encoder
     * @param factor The ratio of encoder units to desired units (ie. units -> in)
     */
    public void setPositionConversionFactor(double factor) {
        encoder.setPositionConversionFactor(factor);
    }
    
    /**
     * Sets the Velocity Conversion Factor for the encoder
     * @param factor The ratio of encoder units to desired units (ie. units/min-> rad/sec)
     */
    public void setVelocityConversionFactor(double factor) {
        encoder.setVelocityConversionFactor(factor);
    }

    /**
     * Sets the encoder position
     * @param pos The new encoder position
     */
    public void setPosition(double pos){
        encoder.setPosition(pos);
    }

    /**
     * Returns the position of the encoder.
     * By default the position is in encoder units, but will return a distance if the Position Conversion Factor has been set.
     */
    public double getPosition(){
        return encoder.getPosition();
    }

    /**
     * Sets the PID values, must be positive
     * @param Kp The proportional gain value
     * @param Ki The integral gain value
     * @param Kd The derivative gain value
     */
    public void setPID(double Kp, double Ki, double Kd, double Ff){  
        pidController.setP(Kp);
        pidController.setI(Ki);
        pidController.setD(Kd);
        pidController.setFF(Ff);
    }

    public String getName() {
        return name;
    }
    
    public String getShortName() {
        return shortName;
    }

    public void set(boolean stop, double speed){
        if(!stop) super.set(speed);
    }
}