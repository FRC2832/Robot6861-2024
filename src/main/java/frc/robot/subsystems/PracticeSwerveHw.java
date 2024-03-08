package frc.robot.subsystems;

import org.livoniawarriors.Logger;
import org.livoniawarriors.swerve.ISwerveDriveIo;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

@SuppressWarnings("removal")
public class PracticeSwerveHw implements ISwerveDriveIo {

    // measuring the robot, we got 13899 counts/rev, theoretical is 13824 counts/rev
    // (L2 gear set at 6.75:1 ratio)
    // needs to be scaled * 39.37 (in/m) / (4"*Pi wheel diameter) / 10 (units per
    // 100ms)
    private static final double COUNTS_PER_METER = 4331.1 / 0.94362; // velocity units
    private static final double VELO_PER_METER = COUNTS_PER_METER * 10; // distance units

    // Swerve corner locations for kinematics
    // trackwidth = 25" /2 = 12.5" converts to 0.3175 meters
    // wheelbase = 25" /2 = 12.5" converts to 0.3175 meters
    private Translation2d[] swervePositions = {
            new Translation2d(0.3175, 0.3175), // convert inches to meters. y is front to back. left front is 1st wheel
            new Translation2d(0.3175, -0.3175), // front right wheel
            new Translation2d(-0.3175, 0.3175), // rear left
            new Translation2d(-0.3175, -0.3175) // rear right
    };

    private String[] moduleNames = {
            "FL",
            "FR",
            "RL",
            "RR"
    };

    private CANSparkMax[] driveMotors;
    private RelativeEncoder[] driveEncoder;
   // private RelativeEncoder[] driveEncoderVelocity;
    private CANSparkMax[] turnMotors;
    private RelativeEncoder[] turnEncoder;
    private CANCoder[] turnSensors;
    private PIDController[] turnPid;

    private double[] correctedAngle;

    public PracticeSwerveHw() {

        // allocate our hardware
        int numMotors = swervePositions.length;
        driveMotors = new CANSparkMax[numMotors];
       driveEncoder = new RelativeEncoder[numMotors];
       // driveEncoderVelocity = new RelativeEncoder[numMotors];
        turnMotors = new CANSparkMax[numMotors];
        turnSensors = new CANCoder[numMotors];
        correctedAngle = new double[numMotors];
        turnEncoder = new RelativeEncoder[numMotors];
        turnPid = new PIDController[numMotors];

        // FL
        driveMotors[0] = new CANSparkMax(11, MotorType.kBrushless);
        turnMotors[0] = new CANSparkMax(12, MotorType.kBrushless);
        turnSensors[0] = new CANCoder(13);
        driveMotors[0].setInverted(true);

        // driveMotors[0].setSmartCurrentLimit(Constants.DRIVETRAIN_MOTOR_CURRENT_LIMIT_AMPS);

        // FR
        driveMotors[1] = new CANSparkMax(21, MotorType.kBrushless);
        turnMotors[1] = new CANSparkMax(22, MotorType.kBrushless);
        turnSensors[1] = new CANCoder(23);

        // RL
        driveMotors[2] = new CANSparkMax(31, MotorType.kBrushless);
        turnMotors[2] = new CANSparkMax(32, MotorType.kBrushless);
        turnSensors[2] = new CANCoder(33);
        driveMotors[2].setInverted(true);

        // RR
        driveMotors[3] = new CANSparkMax(41, MotorType.kBrushless);
        turnMotors[3] = new CANSparkMax(42, MotorType.kBrushless);
        turnSensors[3] = new CANCoder(43);
        // driveMotors[3].setInverted(true);

        // Current Limit for Drive and Turn (20 and 40)
        for (CANSparkMax drive : driveMotors) {
            drive.setSmartCurrentLimit(Constants.DRIVE_MOTOR_PRIMARY_CURRENT_LIMIT);
            drive.setSecondaryCurrentLimit(Constants.DRIVE_MOTOR_SECONDARY_CURRENT_LIMIT);
        }
        for (CANSparkMax turn : turnMotors) {
            turn.setSmartCurrentLimit(Constants.TURN_MOTOR_PRIMARY_CURRENT_LIMIT);
            turn.setSecondaryCurrentLimit(Constants.TURN_MOTOR_SECONDARY_CURRENT_LIMIT);
        }

        for (CANCoder sensor : turnSensors) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);
        }

        // register stuff for logging
        for (int wheel = 0; wheel < numMotors; wheel++) {
            final int wheelFinal = wheel;
            Logger.registerCanSparkMax(moduleNames[wheel] + " Turn", turnMotors[wheel]);
            Logger.registerCanSparkMax(moduleNames[wheel] + " Drive", driveMotors[wheel]);
            Logger.registerCanCoder(moduleNames[wheel] + " Abs", turnSensors[wheel]);
            Logger.registerSensor(moduleNames[wheel] + " Speed, volts", () -> getCornerSpeed(wheelFinal));
            Logger.registerSensor(moduleNames[wheel] + " Turn Pos", () -> getCornerAngle(wheelFinal));
            Logger.registerSensor(moduleNames[wheel] + " Drive Dist", () -> getCornerDistance(wheelFinal));

            // initialize hardware
            turnEncoder[wheel] = turnMotors[wheel].getEncoder();
            driveMotors[wheel].getEncoder().setPositionConversionFactor(1/21.92);
            driveMotors[wheel].getEncoder().setVelocityConversionFactor(1/(21.92 * 60));
            turnPid[wheel] = new PIDController(0.5 / Math.PI, 0, 0.0); // TODO: modify turnPID values, ki was 0.2

            // driveEncoder[wheel] = driveMotors[wheel].getEncoder();
            
        }
        setDriveMotorBrakeMode(true);
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return turnSensors[wheel].getAbsolutePosition();
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnEncoder[wheel].getPosition();
    }

    @Override
    public double getCornerDistance(int wheel) {
        double encoderRotations = driveMotors[wheel].getEncoder().getPosition();
        //double wheelRotations = encoderRotations * 6.75;
        //double circumference = 0.3191858; // meters
        //double distance = wheelRotations * circumference;
        return encoderRotations;
        // return driveMotors[wheel].getEncoder().getPosition() / COUNTS_PER_METER;
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return swervePositions;
    }

    @Override
    public double getCornerSpeed(int wheel) { 
        return driveMotors[wheel].getEncoder().getVelocity();
    }

    @Override
    public String[] getModuleNames() {
        return moduleNames;
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        // set the drive command
        double velPct = swerveModuleState.speedMetersPerSecond / 4.8; 
        SmartDashboard.putNumber("Wheel Velocity ", velPct);
        double velVolts = velPct * 12.0; 

        // driveMotors[wheel].set(velPct);
        // CANSparkBase.ControlType.kDutyCycle ^
        driveMotors[wheel].setVoltage(velVolts);
    

        // set the turn command
        // we need the request to be within the boundaries, not wrap around the 180
        // point
        double turnRequest = MathUtil.inputModulus(swerveModuleState.angle.getDegrees(), correctedAngle[wheel] - 180,
                correctedAngle[wheel] + 180);
        if (Math.abs(correctedAngle[wheel] - turnRequest) < 1) {
            // reset the PID to remove all the I term error so we don't overshoot and
            // rebound
            turnPid[wheel].reset();
        }
        double turnOutput = -turnPid[wheel].calculate(Math.toRadians(correctedAngle[wheel]),
                Math.toRadians(turnRequest));
        turnMotors[wheel].set(turnOutput);

    }

    @Override
    public void setCorrectedAngle(int wheel, double angle) {
        correctedAngle[wheel] = angle;
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        IdleMode mode;

        if (brakeOn) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }

        for (CANSparkMax motor : driveMotors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        IdleMode mode;

        if (brakeOn) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }

        for (CANSparkMax motor : turnMotors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public void updateInputs() {
        // No op
    }

}
