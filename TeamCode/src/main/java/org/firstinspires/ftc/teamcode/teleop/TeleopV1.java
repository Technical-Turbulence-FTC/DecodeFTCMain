package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.hoodDefault;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.Robot;


@Config
@TeleOp

public class TeleopV1 extends LinearOpMode {


    Robot robot;

    Drivetrain drivetrain;

    Intake intake;

    Spindexer spindexer;

    Transfer transfer;

    MultipleTelemetry TELE;

    GamepadEx g1;

    GamepadEx g2;

    public static double defaultSpeed = 1;

    public static double slowMoSpeed = 0.4;

    public static double power = 0.0;

    public static double pos = hoodDefault;

    public boolean all = false;

    public int ticker = 0;

    ToggleButtonReader g1RightBumper;

    ToggleButtonReader g2Circle;

    ToggleButtonReader g2Square;


    ToggleButtonReader g2Triangle;

    ToggleButtonReader g2RightBumper;

    ToggleButtonReader g1LeftBumper;

    ToggleButtonReader g2LeftBumper;

    ToggleButtonReader g2DpadUp;

    ToggleButtonReader g2DpadDown;

    ToggleButtonReader g2DpadRight;

    ToggleButtonReader g2DpadLeft;

    public boolean leftBumper = false;
    public double g1RightBumperStamp = 0.0;

    public double g1LeftBumperStamp = 0.0;


    public double g2LeftBumperStamp = 0.0;

    public static int spindexerPos = 0;

    public boolean green = false;

    Shooter shooter;

    public boolean scoreAll = false;

    MecanumDrive drive ;

    public boolean autotrack = false;

    public int last = 0;
    public int second = 0;

    public double offset = 0.0;

    public static double rIn = 0.59;

    public static double rOut = 0;

    public boolean notShooting = true;

    public boolean circle = false;

    public boolean square = false;

    public boolean tri = false;



    @Override
    public void runOpMode() throws InterruptedException {

        drive  = new MecanumDrive(hardwareMap, teleStart);





        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                FtcDashboard.getInstance().getTelemetry(),
                telemetry
        );

        g1 = new GamepadEx(gamepad1);

        g1RightBumper  = new ToggleButtonReader(
            g1, GamepadKeys.Button.RIGHT_BUMPER
        );

        g2 = new GamepadEx(gamepad2);

        g1LeftBumper = new ToggleButtonReader(
                g1, GamepadKeys.Button.LEFT_BUMPER
        );

        g2Circle  = new ToggleButtonReader(
                g2, GamepadKeys.Button.B
        );

        g2Triangle  = new ToggleButtonReader(
                g2, GamepadKeys.Button.Y
        );

        g2Square  = new ToggleButtonReader(
                g2, GamepadKeys.Button.X
        );

        g2RightBumper  = new ToggleButtonReader(
                g2, GamepadKeys.Button.RIGHT_BUMPER
        );


        g2LeftBumper  = new ToggleButtonReader(
                g2, GamepadKeys.Button.LEFT_BUMPER
        );

        g2DpadUp  = new ToggleButtonReader(
                g2, GamepadKeys.Button.DPAD_UP
        );


        g2DpadDown  = new ToggleButtonReader(
                g2, GamepadKeys.Button.DPAD_DOWN
        );

        g2DpadLeft  = new ToggleButtonReader(
                g2, GamepadKeys.Button.DPAD_LEFT
        );


        g2DpadRight  = new ToggleButtonReader(
                g2, GamepadKeys.Button.DPAD_RIGHT
        );




        drivetrain = new Drivetrain(robot, TELE, g1);

        drivetrain.setMode("Default");

        drivetrain.setDefaultSpeed(defaultSpeed);

        drivetrain.setSlowSpeed(slowMoSpeed);

        intake = new Intake(robot);

        transfer = new Transfer(robot);


        spindexer = new Spindexer(robot, TELE);

        spindexer.setTelemetryOn(true);

        shooter = new Shooter(robot, TELE);

        shooter.setShooterMode("MANUAL");

        robot.rejecter.setPosition(rIn);





        waitForStart();

        if (isStopRequested()) return;

        drive  = new MecanumDrive(hardwareMap, teleStart);


        while(opModeIsActive()){

            drive.updatePoseEstimate();

            TELE.addData("pose", drive.localizer.getPose());

            TELE.addData("heading", drive.localizer.getPose().heading.toDouble());


            TELE.addData("off", offset);


            robot.hood.setPosition(pos);

            g1LeftBumper.readValue();

            if (g1LeftBumper.wasJustPressed()){
                g2LeftBumperStamp = getRuntime();



                spindexer.intakeShake(getRuntime());

                leftBumper = true;
            }

            if (leftBumper){
                double time = getRuntime() - g2LeftBumperStamp;

                if (time < 1.0){
                    robot.rejecter.setPosition(rOut);
                } else {
                    robot.rejecter.setPosition(rIn);
                }

            }




            intake();

            drivetrain.update();

            TELE.update();

            transfer.update();

            g2RightBumper.readValue();

            g2LeftBumper.readValue();

            g2DpadDown.readValue();

            g2DpadUp.readValue();

            if (!scoreAll){
                spindexer.checkForBalls();
            }

            if(g2DpadUp.wasJustPressed()){
                pos -=0.02;
            }

            if(g2DpadDown.wasJustPressed()){
                pos +=0.02;
            }

            g2DpadLeft.readValue();

            g2DpadRight.readValue();

            if(g2DpadLeft.wasJustPressed()){
                offset -=0.02;
            }

            if(g2DpadRight.wasJustPressed()){
                offset +=0.02;
            }



            TELE.addData("hood", pos);








            if (Math.abs(gamepad2.right_stick_x) < 0.1 && autotrack) {




                shooter.trackGoal(drive.localizer.getPose(), new Pose2d(-10, 0, 0), offset);

            } else {

                autotrack = false;

                shooter.moveTurret(0.3+offset);

            }

            if (gamepad2.right_stick_button){
                autotrack = true;
            }



            if (g2RightBumper.wasJustPressed()){
                transfer.setTransferPower(1);
                transfer.transferIn();
                shooter.setManualPower(1);

                notShooting = false;

            }

            if (g2RightBumper.wasJustReleased()){
                transfer.setTransferPower(1);
                transfer.transferOut();
            }

            if (gamepad2.left_stick_y>0.5){

                shooter.setManualPower(0);
            } else if (gamepad2.left_stick_y<-0.5){
                shooter.setManualPower(1);
            }

            if (g2LeftBumper.wasJustPressed()){
                g2LeftBumperStamp = getRuntime();
                notShooting = false;
                scoreAll = true;
            }

            if (scoreAll) {
                double time = getRuntime() - g2LeftBumperStamp;


                shooter.setManualPower(1);

                TELE.addData("greenImportant", green);

                TELE.addData("last", last);
                TELE.addData("2ndLast", second);

                int numGreen = spindexer.greens();

                if (square) {


                    if (time < 0.3) {

                        ticker = 0;

                        last = 0;
                        second = 0;


                        transfer.transferOut();
                        transfer.setTransferPower(1);
                    } else if (time < 2) {

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakePurple(second, last);
                                second = last;
                            } else {
                                last = spindexer.outtakeGreen(second, last);
                                second = last;

                            }
                        }

                        second = last;

                        ticker++;


                    } else if (time < 2.5) {

                        ticker = 0;

                        second = last;


                        transfer.transferIn();
                    } else if (time < 4) {
                        transfer.transferOut();

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakeGreen(second, last);
                            } else {
                                last = spindexer.outtakePurple(second, last);

                            }
                        }

                        ticker++;
                    } else if (time < 4.5) {

                        ticker = 0;


                        transfer.transferIn();
                    } else if (time < 6) {


                        transfer.transferOut();

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakeGreen(second, last);
                            } else {
                                last = spindexer.outtakePurple(second, last);

                            }
                        }

                        ticker++;

                    } else if (time < 6.5) {
                        transfer.transferIn();
                    } else {

                        ticker = 0;


                        scoreAll = false;
                        transfer.transferOut();

                        shooter.setManualPower(0);

                    }
                } else if (tri) {


                    if (time < 0.3) {

                        ticker = 0;

                        last = 0;
                        second = 0;


                        transfer.transferOut();
                        transfer.setTransferPower(1);
                    } else if (time < 2) {

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakeGreen(second, last);
                                second = last;
                            } else {
                                last = spindexer.outtakePurple(second, last);
                                second = last;

                            }
                        }

                        second = last;

                        ticker++;


                    } else if (time < 2.5) {

                        ticker = 0;

                        second = last;


                        transfer.transferIn();
                    } else if (time < 4) {
                        transfer.transferOut();

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakePurple(second, last);
                            } else {
                                last = spindexer.outtakeGreen(second, last);

                            }
                        }

                        ticker++;
                    } else if (time < 4.5) {

                        ticker = 0;


                        transfer.transferIn();
                    } else if (time < 6) {


                        transfer.transferOut();

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakeGreen(second, last);
                            } else {
                                last = spindexer.outtakePurple(second, last);

                            }
                        }

                        ticker++;

                    } else if (time < 6.5) {
                        transfer.transferIn();
                    } else {

                        ticker = 0;


                        scoreAll = false;
                        transfer.transferOut();

                        shooter.setManualPower(0);

                    }
                } else if (circle){


                    if (time < 0.3) {

                        ticker = 0;

                        last = 0;
                        second = 0;


                        transfer.transferOut();
                        transfer.setTransferPower(1);
                    } else if (time < 2) {

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakeGreen(second, last);
                                second = last;
                            } else {
                                last = spindexer.outtakePurple(second, last);
                                second = last;

                            }
                        }

                        second = last;

                        ticker++;


                    } else if (time < 2.5) {

                        ticker = 0;

                        second = last;


                        transfer.transferIn();
                    } else if (time < 4) {
                        transfer.transferOut();

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakeGreen(second, last);
                            } else {
                                last = spindexer.outtakePurple(second, last);

                            }
                        }

                        ticker++;
                    } else if (time < 4.5) {

                        ticker = 0;


                        transfer.transferIn();
                    } else if (time < 6) {


                        transfer.transferOut();

                        if (ticker == 0) {

                            if (numGreen == 2) {
                                last = spindexer.outtakePurple(second, last);
                            } else {
                                last = spindexer.outtakeGreen(second, last);

                            }
                        }

                        ticker++;

                    } else if (time < 6.5) {
                        transfer.transferIn();
                    } else {

                        ticker = 0;


                        scoreAll = false;
                        transfer.transferOut();

                        shooter.setManualPower(0);

                    }
                } else{


                    if (time < 0.3) {

                        ticker = 0;

                        last = 0;
                        second = 0;

                        if (gamepad2.right_trigger > 0.5) {
                            green = false;

                            all = gamepad2.left_trigger > 0.5;

                        } else if (gamepad2.left_trigger > 0.5) {
                            green = true;

                            all = false;
                        } else {
                            all = true;
                        }


                        transfer.transferOut();
                        transfer.setTransferPower(1);
                    } else if (time < 2) {

                        if (ticker == 0) {

                            if (all) {
                                spindexer.outtake3();
                                last = 3;
                                second = 3;
                            } else if (green) {
                                last = spindexer.outtakeGreen(second, last);
                                second = last;
                            } else {
                                last = spindexer.outtakePurple(second, last);
                                second = last;

                            }
                        }

                        second = last;

                        ticker++;


                    } else if (time < 2.5) {

                        ticker = 0;

                        second = last;

                        if (gamepad2.right_trigger > 0.5) {
                            green = false;

                            all = gamepad2.left_trigger > 0.5;

                        } else if (gamepad2.left_trigger > 0.5) {
                            green = true;

                            all = false;


                        }

                        transfer.transferIn();
                    } else if (time < 4) {
                        transfer.transferOut();

                        if (ticker == 0) {

                            if (all) {
                                spindexer.outtake2();

                                last = 2;
                            } else if (green) {
                                last = spindexer.outtakeGreen(second, last);
                            } else {
                                last = spindexer.outtakePurple(second, last);

                            }
                        }

                        ticker++;
                    } else if (time < 4.5) {

                        ticker = 0;


                        if (gamepad2.right_trigger > 0.5) {
                            green = false;

                            all = gamepad2.left_trigger > 0.5;

                        } else if (gamepad2.left_trigger > 0.5) {
                            green = true;

                            all = false;
                        }

                        transfer.transferIn();
                    } else if (time < 6) {


                        transfer.transferOut();

                        if (ticker == 0) {

                            if (all) {
                                spindexer.outtake1();
                            } else if (green) {
                                last = spindexer.outtakeGreen(second, last);
                            } else {
                                last = spindexer.outtakePurple(second, last);

                            }
                        }

                        ticker++;

                    } else if (time < 6.5) {
                        transfer.transferIn();
                    } else {

                        ticker = 0;


                        scoreAll = false;
                        transfer.transferOut();

                        shooter.setManualPower(0);

                    }


                }
            }


            shooter.update();












        }




    }

    public void intake(){


        g1RightBumper.readValue();

        g2Circle.readValue();

        g2Square.readValue();

        g2Triangle.readValue();

        if (g1RightBumper.wasJustPressed()){

            notShooting = true;




            if (getRuntime() - g1RightBumperStamp < 0.3){
                intake.reverse();
            } else {
                intake.toggle();
            }

            if (intake.getIntakeState()==1){
                shooter.setManualPower(0);
            }




            spindexer.intake();
            
            transfer.transferOut();


            g1RightBumperStamp = getRuntime();

        }


        if (intake.getIntakeState()==1 && notShooting) {

            spindexer.intakeShake(getRuntime());

        } else {
            if (g2Circle.wasJustPressed()){
                circle = true;
                tri = false;
                square = false;



            }

            if (g2Triangle.wasJustPressed()){
                circle = false;
                tri = true;
                square = false;
            }

            if (g2Square.wasJustPressed()){
                circle = false;
                tri = false;
                square = true;
            }

            if (gamepad2.x){
                circle = false;
                tri = false;
                square = false;
            }




        }


        intake.update();




        spindexer.update();





    }
}
