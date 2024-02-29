
public class Climber extends SubsystemBase{
    private CCSparkMax climberMotorRight = new CCSparkMax("Climber One", "CO", Constants.MotorConstants.CLIMBER_RIGHT, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.CLIMBER_RIGHT_REVERSED);
    private CCSparkMax climberMotorLeft = new CCSparkMax("Climber Two", "CT", Constants.MotorConstants.CLIMBER_LEFT, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.CLIMBER_LEFT_REVERSED);
    public Climber() {
    }

    public void climb(double speed) {
            
        climberMotorRight.set(speed);
        climberMotorLeft.set(speed);
            }
    }
}