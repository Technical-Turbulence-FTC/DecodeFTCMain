package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.37888)
            .forwardZeroPowerAcceleration(-30.322)
            .lateralZeroPowerAcceleration(-60.876)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.015, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013, 0, 0.00001, 0.6, 0.015))
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(84.376)
            .yVelocity(64.052);

    public static double breakingStrength = 1;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, breakingStrength, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.7795)
            .strafePodX(-3.769)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerBuilder followerBuilder;
        followerBuilder = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants);

        followerBuilder.resetPinpoint();
        followerBuilder.recalibrateIMU();

        return followerBuilder.build();

    }
}
