//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//public class Lift implements Subsystem {
//    Gamepad gamepad1;
//    Gamepad gamepad2;
//    DcMotor liftRight;
//    DcMotor liftLeft;
//
//    public static double intake;
//    public static double levelOne;
//    public static double levelTwo;
//    public static double levelThree;
//    public static double DUMP_TIME;
//
//    ElapsedTime time;
//
//
//    //o
//    //a
//    public State state = State.INTAKE;
//
//    public enum State {INTAKE, EXTEND, DEPOSIT, RETRACT}
//
//    public Lift(Gamepad g1, Gamepad g2) {
//        gamepad1 = g1;
//        gamepad2 = g2;
//    }
//
//    @Override
//    public void init(HardwareMap hw) {
//        liftLeft = hw.get(DcMotor.class, "liftLeft");
//        liftRight = hw.get(DcMotor.class, "liftRight");
//        time = new ElapsedTime();
//    }
//
//    @Override
//    public void update(Robot robot) {
//        switch (state) {
//            case INTAKE:
//                robot.v4b.intake();
//                intakePosition();
//                robot.claw.open();
//                robot.intake.intakeDown();
//                if (gamepad1.x) {
//                    robot.claw.close();
//                    robot.v4b.hold();
//                    depositThree();
//                    state = state.EXTEND;
//                }
//                if (gamepad1.y) {
//                    robot.claw.close();
//                    robot.v4b.hold();
//                    depositTwo();
//                    state = state.EXTEND;
//                }
//                break;
//            case EXTEND:
//                if (Math.abs(liftRight.getCurrentPosition() - levelTwo) < 10) {
//                    robot.v4b.deposit();
//                    if (gamepad1.dpad_left) {
//                        robot.claw.open();
//                    }
//                    time.reset();
//                    state = state.DEPOSIT;
//                }
//                break;
//            case DEPOSIT:
//                if (time.seconds() >= DUMP_TIME) {
//                    // The robot waited long enough, time to start
//                    // retracting the lift
//                    robot.v4b.intake();
//                    depositThree();
//                    state = state.RETRACT;
//                }
//                break;
//            case RETRACT:
//                if (Math.abs(liftRight.getCurrentPosition() - levelOne) < 10) {
//                    state = state.INTAKE;
//                }
//                break;
//            default:
//                // should never be reached, as liftState should never be null
//                state = state.INTAKE;
//        }
//    }
//
//
////    public void intakePosition() {
////        double command = control.update(intake,
////                liftLeft.getCurrentPosition());
////        liftLeft.setPower(command);
////        liftRight.setPower(-command);
////    }
////
////    public void depositOne() {
////        double command = control.update(levelOne,
////                liftLeft.getCurrentPosition());
////        liftLeft.setPower(command);
////        liftRight.setPower(-command);
////    }
////
////    public void depositTwo() {
////        double command = control.update(levelTwo,
////                liftLeft.getCurrentPosition());
////        liftLeft.setPower(command);
////        liftRight.setPower(-command);
////    }
////
////    public void depositThree() {
////        double command = control.update(levelThree,
////                liftLeft.getCurrentPosition());
////        liftLeft.setPower(command);
////        liftRight.setPower(-command);
////    }
//}