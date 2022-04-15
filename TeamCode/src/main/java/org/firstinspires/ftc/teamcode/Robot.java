package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.V4b;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    List<Subsystem> subsystems;

    public Drive drive;
    public Intake intake;
    public Deposit deposit;
    public V4b v4b;
    public BasicLift lift;
    public Vision vision;
    public VisionPipeline visionPipeline;

    public Robot(HardwareMap hw, Gamepad g1, Gamepad g2) {
        subsystems = new ArrayList<>();
        drive = new Drive(g1, g2);
        intake = new Intake(g1, g2);
        lift = new BasicLift(g1, g2);
        deposit = new Deposit(g1, g2);
        v4b = new V4b(g1, g2);

        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(deposit);
        subsystems.add(lift);
        subsystems.add(v4b);
        for (Subsystem s : subsystems) {
            s.init(hw);
        }
    }

    public void update() {
        for (Subsystem s : subsystems) {
            s.update(this);
        }
    }
}