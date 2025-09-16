package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.sql.Wrapper;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;


public class FieldCentricDrive implements Subsystem {

    private Dependency<?> dependencies = Subsystem.DEFAULT_DEPENDENCY
            .and(new SingleAnnotation<>(Mercurial.Attach.class));
    public static final FieldCentricDrive INSTANCE = new FieldCentricDrive();

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

    private FieldCentricDrive() {}

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
    @MustBeDocumented
    @Inherited
    public @interface Attach {}

    //@Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        getFrontLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getBackRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getBackLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFrontRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFrontRight().setDirection(DcMotorSimple.Direction.REVERSE);
        getBackRight().setDirection(DcMotorSimple.Direction.REVERSE);
        getFrontLeft().setDirection(DcMotorSimple.Direction.FORWARD);
        getBackLeft().setDirection(DcMotorSimple.Direction.FORWARD);

        getOdo().setOffsets(-5.4, 0, DistanceUnit.INCH);
        getOdo().setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        getOdo().setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        getOdo().resetPosAndIMU();
    }

    //@Override
    public void cleanup(@NonNull Wrapper opmode) {
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


    //@Override
    public void preUserInitLoopHook(@NonNull Wrapper opmode) {}

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
