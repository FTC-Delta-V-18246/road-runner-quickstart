package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Deposit implements Subsystem {

    Gamepad gamepad1;
    Gamepad gamepad2;
    Servo turret;
    Servo trapDoor;

    public static double doorOpen = 0.7;
    public static double doorClose = 0.45;
    public static double turretNEUTRAL = 0.61;
    public static double turretREDPOS = 0.9;
    public static double turretBLUEPOS = 0.3;

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

        turretNeutral();
        close();
    }

    @Override
    public void update(Robot robot) {
        switch (state) {
            case INTAKE:
                turretNeutral();
                break;
            case NEUTRAL:
                turretNeutral();
                if (gamepad1.x) {
                    open();
                }
                break;
            case REDSHARED:
                turretREDSHARED();
                if (gamepad1.x) {
                    open();
                }
                break;
            case BLUESHARED:
                turretBLUESHARED();
                if (gamepad1.x) {
                    open();
                }
                break;
        }
    }

    public void open() {
        trapDoor.setPosition(doorOpen);
    }

    public void close() {
        trapDoor.setPosition(doorClose);
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
}
