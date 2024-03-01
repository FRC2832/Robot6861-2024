package org.livoniawarriors;

import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_Faults;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

@SuppressWarnings("removal")
public class Logger implements Runnable {
    private static final double VOLTS_PER_PSI = 1.931 / 100; // 2.431V at 100psi

    private static HashMap<String, Object> items = new HashMap<>();
    private static PowerDistribution pdp;
    private static String[] pdpNames;
    private static PneumaticHub ph;
    private static String[] pneumaticNames;
    private static BasePigeon pigeon;
    private static CANSparkMax spark; // used to stop warning about not closing motor, since we really don't...

    private NetworkTable currentTable;
    private NetworkTable commandTable;
    private NetworkTable tempTable;
    private NetworkTable faultTable;
    private NetworkTable stickyTable;
    private NetworkTable sensorTable;
    private NetworkTable canStatusTable;
    private static NetworkTable taskTimings;

    private BooleanPublisher flashDrivePresent;

    private static boolean faultSet;
    private static boolean sfaultSet;
    // solenoids, compressor

    public Logger() {
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DataLog log = DataLogManager.getLog();
        DriverStation.startDataLog(log);

        // create our logging table references
        canStatusTable = NetworkTableInstance.getDefault().getTable("CAN Status");
        currentTable = NetworkTableInstance.getDefault().getTable("Motor Currents");
        tempTable = NetworkTableInstance.getDefault().getTable("Motor Temps");
        commandTable = NetworkTableInstance.getDefault().getTable("Device Commands");
        faultTable = NetworkTableInstance.getDefault().getTable("Device Faults");
        stickyTable = NetworkTableInstance.getDefault().getTable("Device Sticky Faults");
        sensorTable = NetworkTableInstance.getDefault().getTable("Sensors");
        taskTimings = NetworkTableInstance.getDefault().getTable("Task Timings (ms)");
        SmartDashboard.putBoolean("Clear Faults", false);

        // Set the scheduler to log Shuffleboard events for command initialize,
        // interrupt, finish
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        command -> Shuffleboard.addEventMarker(
                                "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        command -> Shuffleboard.addEventMarker(
                                "Command interrupted", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandFinish(
                        command -> Shuffleboard.addEventMarker(
                                "Command finished", command.getName(), EventImportance.kNormal));

        flashDrivePresent = UtilFunctions.getNtPub("/Sensors/Flash Drive Attached", true);
    }

    public void start() {
        // we run at 100ms rate because of all the CAN traffic, especially with the PDP
        // log channels
        UtilFunctions.addPeriodic(this, 0.1, 0);
    }

    public static void registerTalon(String name, BaseTalon talon) {
        items.put(name, talon);
    }

    public static void registerCanSparkMax(String name, CANSparkMax spark) {
        items.put(name, spark);
    }

    public static void registerCanCoder(String name, CANCoder coder) {
        items.put(name, coder);
    }

    public static void registerSensor(String name, DoubleSupplier value) {
        items.put(name, value);
    }

    public static void registerPdp(PowerDistribution pdp, String[] channelNames) {
        Logger.pdp = pdp;
        pdpNames = channelNames;
    }

    public static void registerPneumaticHub(PneumaticHub ph, String[] channelNames) {
        Logger.ph = ph;
        pneumaticNames = channelNames;
    }

    public static void registerLoopTimes(Robot robot) {
        new LoopTimeLogger(robot, taskTimings);
    }

    public static void registerPigeon(BasePigeon pigeon) {
        Logger.pigeon = pigeon;
    }

    public void run() {
        // Print keys
        for (String i : items.keySet()) {
            Object item = items.get(i);

            if (item instanceof BaseTalon) {
                readTalon(i, (BaseTalon) item);
            } else if (item instanceof DoubleSupplier) {
                sensorTable.getEntry(i).setDouble(((DoubleSupplier) item).getAsDouble());
            } else if (item instanceof CANCoder) {
                CANCoder coder = (CANCoder) item;
                sensorTable.getEntry(i + " Angle").setDouble(coder.getAbsolutePosition());
                sensorTable.getEntry(i + " Mag Str").setString(coder.getMagnetFieldStrength().name());

                CANCoderFaults faults = new CANCoderFaults();
                coder.getFaults(faults);
                faultTable.getEntry(i).setString(readFaultStruct(faults));

                CANCoderStickyFaults sFaults = new CANCoderStickyFaults();
                coder.getStickyFaults(sFaults);
                stickyTable.getEntry(i).setString(readFaultStruct(sFaults));
                canStatusTable.getEntry(i).setString(coder.getLastError().name());
            } else if (item instanceof CANSparkMax) {
                spark = (CANSparkMax) item;

                commandTable.getEntry(i).setDouble(spark.getAppliedOutput() * spark.getBusVoltage());
                currentTable.getEntry(i).setDouble(spark.getOutputCurrent());
                faultTable.getEntry(i).setString(readSparkFaults(spark.getFaults()));
                stickyTable.getEntry(i).setString(readSparkFaults(spark.getStickyFaults()));
                tempTable.getEntry(i).setDouble(spark.getMotorTemperature());
                canStatusTable.getEntry(i).setString(spark.getLastError().name());
            } else {
                // unknown table
            }
        }

        if (pdp != null) {
            for (int i = 0; i < pdpNames.length; i++) {
                if (pdpNames[i] != null) {
                    currentTable.getEntry("PDP Current " + pdpNames[i]).setDouble(pdp.getCurrent(i));
                } else {
                    // No op
                }
            }
            sensorTable.getEntry("PDP Voltage").setDouble(pdp.getVoltage());
            sensorTable.getEntry("PDP Total Current").setDouble(pdp.getTotalCurrent());
            tempTable.getEntry("PDP").setDouble(pdp.getTemperature());
            faultTable.getEntry("PDP").setString(readFaultStruct(pdp.getFaults()));
            stickyTable.getEntry("PDP").setString(readFaultStruct(pdp.getStickyFaults()));
            // no CAN status to report
        }

        if (ph != null) {
            int solenoids = ph.getSolenoids();
            for (int i = 0; i < pneumaticNames.length; i++) {
                if (pneumaticNames[i] != null) {
                    commandTable.getEntry(pneumaticNames[i]).setBoolean((solenoids & (1 << i)) == 1);
                }
            }
            double volts = ph.getAnalogVoltage(0);
            sensorTable.getEntry("Pressure Sensor").setDouble((volts - 0.5) / VOLTS_PER_PSI);
            sensorTable.getEntry("Pressure Sensor Voltage").setDouble(volts);
            currentTable.getEntry("Compressor").setDouble(ph.getCompressorCurrent());
            currentTable.getEntry("Solenoids").setDouble(ph.getSolenoidsTotalCurrent());
            faultTable.getEntry("PH").setString(readFaultStruct(ph.getFaults()));
            stickyTable.getEntry("PH").setString(readFaultStruct(ph.getStickyFaults()));
            // no CAN status to report
        }

        if (pigeon != null) {
            double[] pigeonData = new double[3];
            pigeon.getYawPitchRoll(pigeonData);
            sensorTable.getEntry("Pigeon Yaw").setDouble(pigeonData[0]);
            sensorTable.getEntry("Pigeon Pitch").setDouble(pigeonData[1]);
            sensorTable.getEntry("Pigeon Roll").setDouble(pigeonData[2]);

            short[] accelData = new short[3];
            pigeon.getBiasedAccelerometer(accelData);
            sensorTable.getEntry("Pigeon Ax").setDouble(accelData[0] / 16384.);
            sensorTable.getEntry("Pigeon Ay").setDouble(accelData[1] / 16384.);
            sensorTable.getEntry("Pigeon Az").setDouble(accelData[2] / 16384.);

            canStatusTable.getEntry("Pigeon").setString(pigeon.getLastError().name());

            if (pigeon instanceof PigeonIMU) {
                PigeonIMU p1 = (PigeonIMU) pigeon;
                PigeonIMU_Faults p1Faults = new PigeonIMU_Faults();
                p1.getFaults(p1Faults);
                faultTable.getEntry("Pigeon").setString(readFaultStruct(p1Faults));

                p1.getStickyFaults(p1Faults);
                stickyTable.getEntry("Pigeon").setString(readFaultStruct(p1Faults));
            } else if (pigeon instanceof Pigeon2) {
                Pigeon2 p2 = (Pigeon2) pigeon;
                Pigeon2_Faults p2Faults = new Pigeon2_Faults();
                p2.getFaults(p2Faults);
                faultTable.getEntry("Pigeon").setString(readFaultStruct(p2Faults));

                p2.getStickyFaults(p2Faults);
                stickyTable.getEntry("Pigeon").setString(readFaultStruct(p2Faults));
            } else {
                // unknown pigeon
            }
        }

        CANStatus canStatus = RobotController.getCANStatus();
        canStatusTable.getEntry("CAN Bandwidth").setDouble(canStatus.percentBusUtilization);
        canStatusTable.getEntry("CAN Bus Off Count").setDouble(canStatus.busOffCount);
        canStatusTable.getEntry("CAN RX Error Count").setDouble(canStatus.receiveErrorCount);
        canStatusTable.getEntry("CAN Tx Error Count").setDouble(canStatus.transmitErrorCount);
        canStatusTable.getEntry("CAN Tx Full Count").setDouble(canStatus.txFullCount);

        sensorTable.getEntry("Rio 3.3V Voltage").setDouble(RobotController.getVoltage3V3());
        sensorTable.getEntry("Rio 5V Voltage").setDouble(RobotController.getVoltage5V());
        sensorTable.getEntry("Rio 6V Voltage").setDouble(RobotController.getVoltage6V());
        sensorTable.getEntry("Rio 3.3V Current").setDouble(RobotController.getCurrent3V3());
        sensorTable.getEntry("Rio 5V Current").setDouble(RobotController.getCurrent5V());
        sensorTable.getEntry("Rio 6V Current").setDouble(RobotController.getCurrent6V());
        sensorTable.getEntry("Rio CPU Temp").setDouble(RobotController.getCPUTemp());

        checkClearFaults(false);

        Set<String> keys0 = faultTable.getKeys();
        Set<String> stickyKeys0 = stickyTable.getKeys();

        String[] keys = new String[keys0.size()];
        String[] stickyKeys = new String[stickyKeys0.size()];

        keys0.toArray(keys);
        stickyKeys0.toArray(stickyKeys);

        faultSet = false;
        for (String i : keys) {
            String faultName = faultTable.getEntry(i).getString("EROR");
            if (!faultName.equals("Ok")) {
                faultSet = true;
            }
        }

        sfaultSet = false;
        for (String i : stickyKeys) {
            String faultName = stickyTable.getEntry(i).getString("EROR");
            if (!faultName.equals("Ok")) {
                sfaultSet = true;
            }
        }

        boolean flashDriveAttached;
        if (RobotBase.isReal()) {
            flashDriveAttached = Files.exists(Paths.get("/u"));
        } else {
            flashDriveAttached = true;
        }
        flashDrivePresent.set(flashDriveAttached);
    }

    private void readTalon(String name, BaseTalon talon) {
        String faultStr;
        String sFaultStr;
        // reading the raw bits because we know there are faults not in Faults (aka
        // Neutral Brake Current Limit)
        long handle = talon.getHandle();
        int faultBits = MotControllerJNI.GetFaults(handle);
        ErrorCode error = talon.getLastError();
        int sfaultBits = MotControllerJNI.GetStickyFaults(handle);
        ErrorCode error2 = talon.getLastError();

        if (error != ErrorCode.OK) {
            faultStr = error.name();
        } else if (faultBits > 0) {
            faultStr = readTalonFaults(faultBits);
        } else {
            faultStr = "Ok";
        }

        if (error2 != ErrorCode.OK) {
            sFaultStr = error2.name();
        } else if (sfaultBits > 0) {
            sFaultStr = readTalonFaults(sfaultBits);
        } else {
            sFaultStr = "Ok";
        }

        commandTable.getEntry(name).setDouble(talon.getMotorOutputVoltage());
        currentTable.getEntry(name).setDouble(talon.getSupplyCurrent());
        faultTable.getEntry(name).setString(faultStr);
        stickyTable.getEntry(name).setString(sFaultStr);
        tempTable.getEntry(name).setDouble(talon.getTemperature());
        canStatusTable.getEntry(name).setString(talon.getLastError().name());
    }

    private String readTalonFaults(int bits) {
        Faults faults = new Faults();
        String retVal;

        faults.update(bits);
        retVal = readFaultStruct(faults);
        if (retVal.length() == 0) {
            retVal = "Unknown Fault " + bits;
        }

        return retVal;
    }

    private String readFaultStruct(Object obj) {
        StringBuilder work = new StringBuilder();

        // iterate through all the fields and find the boolean ones
        Field[] fieldlist = obj.getClass().getDeclaredFields();
        for (int i = 0; fieldlist.length > i; i++) {
            Field fld = fieldlist[i];
            if (fld.getType().equals(boolean.class)) {
                try {
                    boolean value = fld.getBoolean(obj);
                    if (value) {
                        work.append(fld.getName() + " ");
                    }
                } catch (IllegalArgumentException e) {
                    System.out.println("IllegalFaultReadArg");
                } catch (IllegalAccessException e) {
                    System.out.println("IllegalFaultReadAccess");
                }
            }
        }

        // check if string is empty
        if (work.length() == 0) {
            work.append("Ok");
        }
        return work.toString();
    }

    private String readSparkFaults(short faults) {
        if (faults == 0) {
            return "Ok";
        }
        StringBuilder work = new StringBuilder();
        for (int i = 0; i < 15; i++) {
            if ((faults & (1 << i)) == 1) {
                FaultID fault = CANSparkBase.FaultID.fromId(i);
                work.append(fault.name()).append(" ");
            }
        }
        return work.toString();
    }

    public static void checkClearFaults(boolean clear) {
        boolean clearFaults = SmartDashboard.getBoolean("Clear Faults", false);

        if (!clearFaults && !clear) {
            return;
        }
        SmartDashboard.putBoolean("Clear Faults", false);

        for (String name : items.keySet()) {
            Object item = items.get(name);
            if (item instanceof BaseTalon) {
                BaseTalon talon = (BaseTalon) item;
                talon.clearStickyFaults();
            } else if (item instanceof CANCoder) {
                CANCoder coder = (CANCoder) item;
                coder.clearStickyFaults();
            } else if (item instanceof CANSparkMax) {
                spark = (CANSparkMax) item;
                spark.clearFaults();
            } else {
                // unknown table
            }
        }

        if (pdp != null) {
            pdp.clearStickyFaults();
        }

        if (ph != null) {
            ph.clearStickyFaults();
        }

        if (pigeon != null) {
            pigeon.clearStickyFaults();
        }
    }

    public static boolean getFaultSet() {
        return faultSet;
    }

    public static boolean getStickyFaultSet() {
        return sfaultSet;
    }
}
