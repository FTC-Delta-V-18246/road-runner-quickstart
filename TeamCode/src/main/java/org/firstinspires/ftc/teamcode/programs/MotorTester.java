package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
@Config
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_up) {
                arm.setPower(0.1);
            } else {
                arm.setPower(0);
            }

            if(gamepad1.dpad_down) {
                arm.setPower(-0.1);
            } else {
                arm.setPower(0);
            }
        }
    }
}
