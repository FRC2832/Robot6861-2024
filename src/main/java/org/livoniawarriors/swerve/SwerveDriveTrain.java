package org.livoniawarriors.swerve;

import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.odometry.Odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
    /** The fastest rate we want the drive wheels to change speeds in m/s */
    static final String MAX_ACCEL_KEY = "Swerve Drive/Max Wheel Accel";
    /** The fastest rate we want the swerve wheels to turn in deg/s */
    static final String MAX_OMEGA_KEY = "Swerve Drive/Max Wheel Omega";
    /** The max speed possible with the swerve wheels in m/s */
    static final String MIN_SPEED_KEY = "Swerve Drive/Min Speed";
    static final String MAX_SPEED_KEY = "Swerve Drive/Max Speed";
    /**
     * The angle in degrees we want the swerve to invert the request to get to
     * position faster
     */
    static final String OPTIMIZE_ANGLE_KEY = "Swerve Drive/Optimize Angle";
    static final String FIELD_ORIENTED = "Swerve Drive/Field Oriented";

    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swervePositions;
    private SwerveModuleState[] swerveTargets;
    private double gyroOffset = 0.0;
    private PIDController pidZero = new PIDController(.0000, 0.0000, 0); 
                   // pidZero is not working as expected. Keep all gains = 0 as of 8/9/24. Code below needs rewrite.
    private SwerveModuleState[] swerveStates;
    private boolean optimize;
    private boolean resetZeroPid;
    private double minSpeed;
    private double maxSpeed;
    private Rotation2d currentHeading;
    private Rotation2d fieldOffset;
    private Odometry odometry;
    private boolean lastTeleop;

    // input settings
    private DoubleSubscriber driverMaxSpeed;
    private DoubleSubscriber driverMaxOmega;
    private DoubleSubscriber[] wheelOffsetSetting;

    // output data
    private DoublePublisher[] wheelCalcAngle;
    private DoublePublisher[] wheelCommandAngle;
    private DoublePublisher[] wheelRequestAngle;
    private DoublePublisher[] wheelCommandSpeed;
    private DoublePublisher[] wheelRequestSpeed;
    private DoublePublisher swerveXSpeed;
    private DoublePublisher swerveYSpeed;
    private DoublePublisher swerveOmega;
    private DoubleArrayPublisher swerveStatePub;
    private DoubleArrayPublisher swerveRequestPub;

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo, Odometry odometry) {
        super();
        this.hardware = hSwerveDriveIo;
        this.odometry = odometry;
        optimize = true;
        resetZeroPid = false;
        int numWheels = hardware.getCornerLocations().length;
        fieldOffset = new Rotation2d();

        // initialize module names
        String[] moduleNames = hardware.getModuleNames();

        // initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());

        // initialize the swerve states
        swervePositions = new SwerveModulePosition[numWheels];
        swerveTargets = new SwerveModuleState[numWheels];
        swerveStates = new SwerveModuleState[numWheels];
        wheelOffsetSetting = new DoubleSubscriber[numWheels];
        wheelCalcAngle = new DoublePublisher[numWheels];
        wheelCommandAngle = new DoublePublisher[numWheels];
        wheelRequestAngle = new DoublePublisher[numWheels];
        wheelCommandSpeed = new DoublePublisher[numWheels];
        wheelRequestSpeed = new DoublePublisher[numWheels];
        double[] wheelOffsetSettingBackups = { 18.19, 287.60, 343.35, 14.326 };
        for (int wheel = 0; wheel < numWheels; wheel++) {
            swervePositions[wheel] = new SwerveModulePosition();
            swerveTargets[wheel] = new SwerveModuleState();
            swerveStates[wheel] = new SwerveModuleState();
            wheelOffsetSetting[wheel] = UtilFunctions
                    .getSettingSub("/Swerve Drive/Wheel Offset " + moduleNames[wheel] + " (deg)",
                            wheelOffsetSettingBackups[wheel]);
            wheelCalcAngle[wheel] = UtilFunctions
                    .getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Calc Angle (deg)", 0);
            wheelCommandAngle[wheel] = UtilFunctions
                    .getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Command Angle (deg)", 0);
            wheelRequestAngle[wheel] = UtilFunctions
                    .getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Request Angle (deg)", 0);
            wheelCommandSpeed[wheel] = UtilFunctions
                    .getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Command Speed (mps)", 0);
            wheelRequestSpeed[wheel] = UtilFunctions
                    .getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Request Speed (mps)", 0);
        }

        /** How fast we want the driver to go during normal operation in m/s */
        driverMaxSpeed = UtilFunctions.getSettingSub("/Swerve Drive/Max Driver Speed (mps)", 4.9);  //TODO: was 4.3 end of Livonia comp.  turned down for new driver practice
        /** How fast we want the driver to turn during normal operation in deg/s */
        driverMaxOmega = UtilFunctions.getSettingSub("/Swerve Drive/Max Driver Omega (dps)", 525); // TODO: was 625 end of livonia comp. turned down for new driver practice  1.8 * Pi rad/sec

        minSpeed = 0.5;  //units m/s
        maxSpeed = 4.8;  //units m/s

        swerveXSpeed = UtilFunctions.getNtPub("/Swerve Drive/X Speed (mps)", 0);
        swerveYSpeed = UtilFunctions.getNtPub("/Swerve Drive/Y Speed (mps)", 0);
        swerveOmega = UtilFunctions.getNtPub("/Swerve Drive/Omega (dps)", 0);
        swerveStatePub = UtilFunctions.getNtPub("/Swerve Drive/Module States", new double[0]);
        swerveRequestPub = UtilFunctions.getNtPub("/Swerve Drive/Module Requests", new double[0]);
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        currentHeading = odometry.getGyroRotation();

        // read the swerve corner state
        for (int wheel = 0; wheel < swervePositions.length; wheel++) {
            double offset = wheelOffsetSetting[wheel].get(0);
            double angle = hardware.getCornerAbsAngle(wheel) - offset;
            swervePositions[wheel].angle = Rotation2d.fromDegrees(angle);
            swervePositions[wheel].distanceMeters = hardware.getCornerDistance(wheel);

            swerveStates[wheel].angle = swervePositions[wheel].angle;
            //if hardware.getCornerSpeed(wheel) < 
            swerveStates[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);

            wheelCalcAngle[wheel].set(angle);
            hardware.setCorrectedAngle(wheel, angle);


        }

        // when we are disabled, reset the turn pids as we don't want to act on the
        // "error" when reenabled
        boolean curTeleop = DriverStation.isTeleopEnabled();
        if (!lastTeleop && curTeleop || resetZeroPid) {
            gyroOffset = currentHeading.getDegrees();
            fieldOffset = currentHeading;
            pidZero.reset();
        }
        lastTeleop = curTeleop;
        resetZeroPid = false;

        pushSwerveStates(swerveStates, swerveTargets);

        
        minSpeed = UtilFunctions.getSetting(MIN_SPEED_KEY, 0.5); 
                                                                 
        maxSpeed = UtilFunctions.getSetting(MAX_SPEED_KEY, 4.6);  //increase to 4.8 when experienced drivers are driving
    }

    /**
     * Used by PathPlanner to set a robot command
     * 
     * @param speeds Speeds to use
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    public void swerveDrive(double xSpeed, double ySpeed, double turn) {
        swerveDrive(xSpeed, ySpeed, turn, UtilFunctions.getSetting(FIELD_ORIENTED, true));
    }

    public void swerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;

        if (Math.abs(turn) > 0.1) {
            // if a turn is requested, reset the zero for the drivetrain
            gyroOffset = currentHeading.getDegrees();
            pidZero.reset();
        } else {
            // straighten the robot
            turn = pidZero.calculate(currentHeading.getDegrees(), gyroOffset);
            SmartDashboard.putNumber("gyro Offset ", gyroOffset);
            //SmartDashboard.putNumber("turn ", turn);
        }

        // try this? to fix odd flipping in requested angle/speed that seems to be connected to tiny turn value flipping +/-
        if (Math.abs(turn) < 0.006) { 
            turn = 0.00; 
            //System.out.println("debugging: had to set turn to 0");
        } 
        SmartDashboard.putNumber("turn ", turn); 



        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, currentHeading.minus(fieldOffset));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }

        // calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, maxSpeed);

        // log the request
        swerveXSpeed.set(xSpeed);
        swerveYSpeed.set(ySpeed);
        swerveOmega.set(Math.toDegrees(turn));


        for (int i = 0; i < requestStates.length; i++) {
            wheelRequestAngle[i].set(requestStates[i].angle.getDegrees());
            wheelRequestSpeed[i].set(requestStates[i].speedMetersPerSecond);
        }

        // filter the swerve wheels
        if (optimize) {
            requestStates = optimizeSwerve(requestStates, swerveStates, true);
        }
        setCornerStates(requestStates);
    }

    public void setWheelCommand(SwerveModuleState[] requestStates) {
        resetZeroPid = true;

        // command the hardware
        if (optimize) {
            requestStates = optimizeSwerve(requestStates, swerveStates, false);
        }
        setCornerStates(requestStates);

        // log the request
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(requestStates);
        swerveXSpeed.set(speeds.vxMetersPerSecond);
        swerveYSpeed.set(speeds.vyMetersPerSecond);
        swerveOmega.set(Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    public SwerveModuleState[] optimizeSwerve(SwerveModuleState[] requestStates, SwerveModuleState[] currentState,
            boolean stopTurnAtZero) {
        SwerveModuleState[] outputStates = new SwerveModuleState[requestStates.length];
        // we use a little larger optimize angle since drivers turning 90* is a pretty
        // common operation
        double optimizeAngle = UtilFunctions.getSetting(OPTIMIZE_ANGLE_KEY, 120);
        double maxAccel = UtilFunctions.getSetting(MAX_ACCEL_KEY, 625);  
        double maxOmega = UtilFunctions.getSetting(MAX_OMEGA_KEY, 3000);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            outputStates[i] = new SwerveModuleState();

            // figure out if we should invert the request
            double angleReq = requestStates[i].angle.getDegrees();
            double curAngle = currentState[i].angle.getDegrees();
            double speedReq = requestStates[i].speedMetersPerSecond;
            double deltaMod = MathUtil.inputModulus(angleReq - curAngle, -180, 180);
            if (Math.abs(deltaMod) > optimizeAngle) {
                angleReq = angleReq - 180;
                speedReq = -requestStates[i].speedMetersPerSecond;
            }

            // smooth out drive command
            double maxSpeedDelta = maxAccel * TimedRobot.kDefaultPeriod; // acceleration * loop time
            // whatever value is bigger flips when forwards vs backwards
            double value1 = currentState[i].speedMetersPerSecond - maxSpeedDelta;
            double value2 = currentState[i].speedMetersPerSecond + maxSpeedDelta;
            

            outputStates[i].speedMetersPerSecond = MathUtil.clamp(
                    speedReq, // current request
                    Math.min(value1, value2), // last request minimum
                    Math.max(value1, value2)); // last request maximum






            // smooth out turn command
            double maxAngleDelta = maxOmega * TimedRobot.kDefaultPeriod; // acceleration * loop time
            angleReq = MathUtil.inputModulus(angleReq, curAngle - 180, curAngle + 180);
            double delta = angleReq - curAngle;
            if (delta > maxAngleDelta) {
                angleReq = curAngle + maxAngleDelta;
            } else if (delta < -maxAngleDelta) {
                angleReq = curAngle - maxAngleDelta;
            } else {
                // angle request if fine
            }
            // make it back into a Rotation2d
            outputStates[i].angle = Rotation2d.fromDegrees(angleReq);

            // check to see if the robot request is moving
            if (stopTurnAtZero && Math.abs(speedReq) < minSpeed) {
                // stop the requests if there is no movement
                outputStates[i].angle = currentState[i].angle;
                // take out minimal speed so that the motors don't jitter
                outputStates[i].speedMetersPerSecond = 0;
            }
        }
        return outputStates;
    }

    private void pushSwerveStates(SwerveModuleState[] state, SwerveModuleState[] request) {
        int size = state.length;
        double[] states = new double[size * 2];
        double[] requests = new double[size * 2];
        for (int i = 0; i < size; i++) {
            states[(i * 2)] = state[i].angle.getDegrees();
            states[(i * 2) + 1] = state[i].speedMetersPerSecond;
            requests[(i * 2)] = request[i].angle.getDegrees();
            requests[(i * 2) + 1] = request[i].speedMetersPerSecond;
        }
        swerveStatePub.set(states);
        swerveRequestPub.set(requests);
    }

    public void stopWheels() {
        swerveDrive(0, 0, 0);
    }

    public void resetFieldOriented() {
        odometry.zeroGyroAngle();
        fieldOffset = odometry.getPose().getRotation().minus(odometry.getGyroRotation());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getSwervePositions() {
        return swervePositions;
    }

    public SwerveModuleState[] getSwerveStates() {
        return swerveStates;
    }

    public void setTurnMotorBrakeMode(boolean brakeOn) {
        hardware.setTurnMotorBrakeMode(brakeOn);
    }

    public void setDriveMotorBrakeMode(boolean brakeOn) {
        hardware.setDriveMotorBrakeMode(brakeOn);
    }

    public Translation2d[] getCornerLocations() {
        return hardware.getCornerLocations();
    }

    public void setOptimizeOn(boolean enabled) {
        optimize = enabled;
    }

    public boolean getOptimizeOn() {
        return optimize;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxDriverSpeed() {
        return driverMaxSpeed.get();
    }

    public double getMaxDriverOmega() {
        return Math.toRadians(driverMaxOmega.get());
    }

    public double getMinSpeed() {
        return minSpeed;
    }

    private void setCornerStates(SwerveModuleState[] states) {
        swerveTargets = states;
        for (int wheel = 0; wheel < states.length; wheel++) {
            hardware.setCornerState(wheel, states[wheel]);

            // log the commands
            wheelCommandAngle[wheel].set(states[wheel].angle.getDegrees());
            wheelCommandSpeed[wheel].set(states[wheel].speedMetersPerSecond);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(swerveStates);
    }

    /**
     * Drive base radius in meters. Distance from robot center to furthest module.
     * 
     * @return Distance in meters.
     */
    public double getDriveBaseRadius() {
        double dist = 0;

        for (Translation2d module : hardware.getCornerLocations()) {
            double newDist = Math.sqrt((module.getX() * module.getX()) + (module.getY() * module.getY()));
            dist = Math.max(newDist, dist);
        }

        return dist;
    }
}
