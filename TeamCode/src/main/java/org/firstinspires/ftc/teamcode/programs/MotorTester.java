package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.BasicLift;

@TeleOp
@Config
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicLift lift = new BasicLift(gamepad1, gamepad2);
        lift.init(hardwareMap);

        waitForStart();


        while(opModeIsActive()) {
            //BasicLift.update();
        }
    }
}
