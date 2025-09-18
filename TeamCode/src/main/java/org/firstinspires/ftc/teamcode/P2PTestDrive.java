package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponentSupplier;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import dev.frozenmilk.util.units.angle.Angle;
import dev.frozenmilk.util.units.angle.Angles;
import dev.frozenmilk.util.units.distance.Distance;
import dev.frozenmilk.util.units.distance.DistanceUnits;
import dev.frozenmilk.util.units.position.DistancePose2D;
import dev.frozenmilk.util.units.position.DistanceVector2D;

public class P2PTestDrive implements Subsystem {

    private Dependency<?> dependencies = Subsystem.DEFAULT_DEPENDENCY
            .and(new SingleAnnotation<>(Mercurial.Attach.class));
    public static final P2PTestDrive INSTANCE = new P2PTestDrive();

    private final SubsystemObjectCell<CachedMotor> frontLeft = subsystemCell(() ->
            new CachedMotor(FeatureRegistrar.getActiveOpMode().hardwareMap, "frontLeftMotor"));
    private final SubsystemObjectCell<CachedMotor> frontRight = subsystemCell(() ->
            new CachedMotor(FeatureRegistrar.getActiveOpMode().hardwareMap, "frontRightMotor"));
    private final SubsystemObjectCell<CachedMotor> backLeft = subsystemCell(() ->
            new CachedMotor(FeatureRegistrar.getActiveOpMode().hardwareMap, "backLeftMotor"));
    private final SubsystemObjectCell<CachedMotor> backRight = subsystemCell(() ->
            new CachedMotor(FeatureRegistrar.getActiveOpMode().hardwareMap, "backRightMotor"));
    private final SubsystemObjectCell<GoBildaPinpointDriver> odo = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "odo"));

    private P2PTestDrive() {}

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependencies;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependencies = dependency;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}

    @Override
    public void preUserInitHook(@NonNull dev.frozenmilk.dairy.core.wrapper.Wrapper opMode) {
        getFrontLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getBackRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getBackLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFrontRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFrontRight().setDirection(DcMotorSimple.Direction.FORWARD);
        getBackRight().setDirection(DcMotorSimple.Direction.FORWARD);
        getFrontLeft().setDirection(DcMotorSimple.Direction.REVERSE);
        getBackLeft().setDirection(DcMotorSimple.Direction.REVERSE);

        getOdo().setOffsets(-5.4, 0, DistanceUnit.INCH);
        getOdo().setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        getOdo().setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        getOdo().resetPosAndIMU();
    }

    @Override
    public void cleanup(@NonNull dev.frozenmilk.dairy.core.wrapper.Wrapper opmode) {
        frontLeft.invalidate();
        backLeft.invalidate();
        frontRight.invalidate();
        backRight.invalidate();
    }

    public CachedMotor getFrontLeft() { return frontLeft.get(); }
    public CachedMotor getBackLeft() { return backLeft.get(); }
    public CachedMotor getFrontRight() { return frontRight.get(); }
    public CachedMotor getBackRight() { return backRight.get(); }
    public GoBildaPinpointDriver getOdo() { return odo.get(); }

    @Override
    public void preUserInitLoopHook(@NonNull dev.frozenmilk.dairy.core.wrapper.Wrapper opmode) {}


    private final DistancePose2D targetPose = new DistancePose2D(
            new DistanceVector2D(DistanceUnits.MILLIMETER, 10000, 100),
            Angles.deg(5)
    );

    private final P2PFeedforward controller = new P2PFeedforward(
            0.2, // kP
            0.005, // kD
            0.1 // kV (start small, tune)
    ); // Tune this!

    private boolean goToPoint = false;
    private long lastUpdateTime = System.nanoTime();

    public Lambda driveToPointCommand(BoundGamepad gamepad) {
        return new Lambda("go-to-point")
                .setInit(() -> {
                    controller.reset();
                    goToPoint = false;
                })
                .setExecute(() -> {
                    getOdo().update();

                    gamepad.a().onTrue(
                            new Lambda("goToPoint")
                                    .setInit(() -> {
                                        goToPoint = true;
                                        controller.reset();
                                    })
                    );




                    DistancePose2D currentPose = new DistancePose2D(
                            new DistanceVector2D(new Distance(DistanceUnits.MILLIMETER, getOdo().getPosX(DistanceUnit.MM)), new Distance(DistanceUnits.MILLIMETER, getOdo().getPosY(DistanceUnit.MM))),
                            Angles.deg(getOdo().getHeading(AngleUnit.DEGREES)));

                    if (goToPoint) {
                        long currentTime = System.nanoTime();
                        double deltaTime = (currentTime - lastUpdateTime) / 1e9;
                        lastUpdateTime = currentTime;

                        MotionComponentSupplier<DistancePose2D> state = (motionComponents) -> currentPose;
                        MotionComponentSupplier<DistancePose2D> target = (motionComponents) -> targetPose;
                        MotionComponentSupplier<DistancePose2D> error = (motionComponents) -> targetPose.minus(currentPose);

                        controller.update(new DistancePose2D(), state, target, error, deltaTime);
                        DistancePose2D output = controller.evaluate(new DistancePose2D(), state, target, error, deltaTime);

                        double xPower = output.getVector2D().getX().getValue();
                        double yPower = output.getVector2D().getY().getValue();
                        double turnPower = output.getHeading().getValue();

                        // Normalize powers if necessary
                        double max = Math.max(Math.abs(xPower) + Math.abs(yPower) + Math.abs(turnPower), 1);
                        xPower /= max;
                        yPower /= max;
                        turnPower /= max;

                        // Basic field-centric correction
                        double botHeading = currentPose.getHeading().getValue(); // radians
                        double rotX = xPower * Math.cos(-botHeading) - yPower * Math.sin(-botHeading);
                        double rotY = xPower * Math.sin(-botHeading) + yPower * Math.cos(-botHeading);

                        rotX *= 1.1; // strafe correction

                        double fl = rotY + rotX + turnPower;
                        double bl = rotY - rotX + turnPower;
                        double fr = rotY - rotX - turnPower;
                        double br = rotY + rotX - turnPower;

                        getFrontLeft().setPower(fl);
                        getBackLeft().setPower(bl);
                        getFrontRight().setPower(fr);
                        getBackRight().setPower(br);

                        // Stop if we're close enough
                        if (targetPose.minus(currentPose).getVector2D().getMagnitude().getValue() < 50 &&
                                Math.abs(targetPose.getHeading().getValue() - currentPose.getHeading().getValue()) < Math.toRadians(5)) {
                            getFrontLeft().setPower(0);
                            getBackLeft().setPower(0);
                            getFrontRight().setPower(0);
                            getBackRight().setPower(0);
                            goToPoint = false;
                        }
                    }
                })
                .setFinish(() -> false);
    }

    public Lambda robotCentricDriveCommand(BoundGamepad gamepad) {
        return new Lambda("mec-drive-robo-centric")
                .setInit(() -> {
                    getFrontRight().setDirection(DcMotorSimple.Direction.REVERSE);
                    getBackRight().setDirection(DcMotorSimple.Direction.REVERSE);
                    getFrontLeft().setDirection(DcMotorSimple.Direction.FORWARD);
                    getBackLeft().setDirection(DcMotorSimple.Direction.FORWARD);
                })
                .setExecute(() -> {
                    getOdo().update();
                    double y = -gamepad.leftStickY().state(); //y
                    double x = -gamepad.leftStickX().state();
                    double rx = -gamepad.rightStickX().state(); //rx

                    double botHeading = getOdo().getHeading(AngleUnit.RADIANS);
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * 1.1;

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    getFrontLeft().setPower(frontLeftPower);
                    getFrontRight().setPower(frontRightPower);
                    getBackRight().setPower(backRightPower);
                    getBackLeft().setPower(backLeftPower);
                })
                .setFinish(() -> false);
    }
}
