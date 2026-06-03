package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;


public class Shooter {

    Robot robot;
    Flywheel fly;
    Turret turr;
    VelocityCommander commander;

    double goalX = 0.0;
    double goalY = 0.0;
    double obeliskX = 72;
    double obeliskY = 144;

    private boolean red = true;


    Follower follow;

    public Shooter(Robot rob, MultipleTelemetry TELE, Follower follower, boolean redAlliance, Turret turret, Flywheel flywheel) {
        this.robot = rob;
        this.fly = flywheel;
        this.turr = turret;
        this.follow = follower;
        this.commander = new VelocityCommander();

        setRedAlliance(redAlliance);

    }

    public void setRedAlliance(boolean input) {
        this.red = input;

        if (this.red) {
            goalX = 144;
        } else {
            goalX = 0;
        }
        goalY = 144;
    }

    private double flywheelVelocity = 0.0;
    private double turretPosition = 0.5;

    public enum ShooterState {
        READ_OBELISK,
        TRACK_GOAL,
        MANUAL_FLYWHEEL_TRACK_TURR,
        MANUAL_TURRET_TRACK_FLY,
        MANUAL,
        NOTHING
    }


    private ShooterState state = ShooterState.NOTHING;

    public void setState(ShooterState shooterState) {
        this.state = shooterState;
    }

    public void setTurretPosition(double input) {
        this.turretPosition = input;
    }

    public void setFlywheelVelocity(double input) {
        this.flywheelVelocity = input;
    }

    public int getObeliskID() {
        return turr.getObeliskID();
    }


    public void update() {

        switch (state) {
            case NOTHING:
                break;
            case MANUAL:
                fly.manageFlywheel(flywheelVelocity);
                turr.manual(turretPosition);
                break;
            case TRACK_GOAL:
                turr.trackGoal(
                        (goalX - follow.getPose().getX()),
                        (goalY - follow.getPose().getY()),
                        follow.getHeading(),
                        follow.getAngularVelocity(),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                flywheelVelocity = commander.getVeloPredictive(
                        (goalX - follow.getPose().getX()),
                        (goalY - follow.getPose().getY()),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                fly.manageFlywheel(flywheelVelocity);
                break;
            case READ_OBELISK:
                turr.trackObelisk(
                        (goalX - follow.getPose().getX()),
                        (goalY - follow.getPose().getY()),
                        follow.getHeading()
                );

                flywheelVelocity = commander.getVeloPredictive(
                        (goalX - follow.getPose().getX()),
                        (goalY - follow.getPose().getY()),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                fly.manageFlywheel(flywheelVelocity);
                break;

            case MANUAL_TURRET_TRACK_FLY:
                turr.manual(turretPosition);
                flywheelVelocity = commander.getVeloPredictive(
                        (goalX - follow.getPose().getX()),
                        (goalY - follow.getPose().getY()),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                fly.manageFlywheel(flywheelVelocity);
                break;

            case MANUAL_FLYWHEEL_TRACK_TURR:
                turr.trackGoal(
                        (goalX - follow.getPose().getX()),
                        (goalY - follow.getPose().getY()),
                        follow.getHeading(),
                        follow.getAngularVelocity(),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );
                fly.manageFlywheel(flywheelVelocity);
                break;

        }

    }

}
