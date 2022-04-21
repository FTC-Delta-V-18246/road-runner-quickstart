package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Deposit implements Subsystem {

    Gamepad gamepad1;
    Gamepad gamepad2;
    Servo turret;
    Servo trapDoor;
    DistanceSensor ssensor;

    public static double close = 0.48;
    public static double kick = 0.0;
    public static double doorCloseDuck = 0.635;
    public static double receive = 0.3;
    public static double turretNEUTRAL = 0.6;
    public static double turretREDPOS = 0.4;
    public static double turretBLUEPOS = 0.8;
    public static double distanceMin = 0;
    public static double distanceMax = 3;
    public static double depositSensor = 5;
    public static double turretAdjust = 0;

    public State state = State.INTAKE;
    public enum State {INTAKE, NEUTRAL, REDSHARED, BLUESHARED}

    public Deposit(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        turret = hw.get(Servo.class, "turret");
        trapDoor = hw.get(Servo.class, "trapDoor");
        ssensor = hw.get(DistanceSensor.class, "sensor");
        turretNeutral();

    }

    @Override
    public void update(Robot robot) {
//        switch (state) {
//            case INTAKE:
//                turretNeutral();
//                break;
//            case NEUTRAL:
//                turretNeutral();
//                if (gamepad1.x) {
//                    open();
//                }
//                break;
//            case REDSHARED:
//                turretREDSHARED();
//                if (gamepad1.x) {
//                    open();
//                }
//                break;
//            case BLUESHARED:
//                turretBLUESHARED();
//                if (gamepad1.x) {
//                    open();
//                }
//                break;;
//        }
    }
    public void receive() {
        trapDoor.setPosition(receive);
    }

    public void close() {
        trapDoor.setPosition(close);
    }
    public void closeDuck() {
        trapDoor.setPosition(doorCloseDuck);
    }
    public void kick() {
        trapDoor.setPosition(kick);
    }

    public void turretNeutral() {
        turret.setPosition(turretNEUTRAL);
    }

    public void turretREDSHARED() {
        turret.setPosition(turretREDPOS);
    }

    public void turretBLUESHARED() {
        turret.setPosition(turretBLUEPOS);
    }

    public void ttelemetry(LinearOpMode opMode) {
        opMode.telemetry.addData("", ssensor.getDistance(DistanceUnit.INCH));
    }
}
