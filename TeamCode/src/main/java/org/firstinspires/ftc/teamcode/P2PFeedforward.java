package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import dev.frozenmilk.dairy.core.util.controller.calculation.ControllerCalculation;
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponentSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents;
import dev.frozenmilk.util.units.angle.AngleUnits;
import dev.frozenmilk.util.units.angle.Angles;
import dev.frozenmilk.util.units.angle.Wrapping;
import dev.frozenmilk.util.units.distance.DistanceUnits;
import dev.frozenmilk.util.units.distance.Distances;
import dev.frozenmilk.util.units.position.DistancePose2D;
import dev.frozenmilk.util.units.position.DistanceVector2D;

public class P2PFeedforward implements ControllerCalculation<DistancePose2D> {
    private final double kP, kD, kV;
    private final DistancePose2D zero = new DistancePose2D();
    private DistancePose2D previousError = zero;

    public P2PFeedforward(double kP, double kD, double kV) {
        this.kP = kP;
        this.kD = kD;
        this.kV = kV;
    }

    @Override
    public void update(
            @NonNull DistancePose2D accumulation,
            @NonNull MotionComponentSupplier<? extends DistancePose2D> state,
            @NonNull MotionComponentSupplier<? extends DistancePose2D> target,
            @NonNull MotionComponentSupplier<? extends DistancePose2D> error,
            double deltaTime
    ) {
        previousError = error.get(MotionComponents.STATE).into(
                DistanceUnits.MILLIMETER,
                DistanceUnits.MILLIMETER,
                AngleUnits.RADIAN,
                Wrapping.LINEAR
        );
    }

    @NonNull
    @Override
    public DistancePose2D evaluate(
            @NonNull DistancePose2D accumulation,
            @NonNull MotionComponentSupplier<? extends DistancePose2D> state,
            @NonNull MotionComponentSupplier<? extends DistancePose2D> target,
            @NonNull MotionComponentSupplier<? extends DistancePose2D> error,
            double deltaTime
    ) {
        DistancePose2D res = accumulation;

        DistancePose2D err = error.get(MotionComponents.STATE).into(
                DistanceUnits.MILLIMETER,
                DistanceUnits.MILLIMETER,
                AngleUnits.RADIAN,
                Wrapping.LINEAR
        );

        DistancePose2D resP = err.times(kP);
        DistancePose2D resD = ((err.minus(previousError)).div(deltaTime)).times(kD);

        // Desired direction (unit vector towards target)
        DistanceVector2D vec = err.getVector2D();
        double x = vec.getX().get(DistanceUnits.MILLIMETER);
        double y = vec.getY().get(DistanceUnits.MILLIMETER);
        double magnitude = Math.hypot(x, y);

// Prevent divide-by-zero
        double normX = magnitude != 0 ? x / magnitude : 0;
        double normY = magnitude != 0 ? y / magnitude : 0;

// Create a new normalized DistanceVector2D
        DistanceVector2D normVec = new DistanceVector2D(
                Distances.mm(normX),
                Distances.mm(normY)
        );

// Normalize heading (just clamp to -1 to 1 directionally)
        double headingError = err.getHeading().intoRadians().getValue();
        double normHeading = headingError == 0 ? 0 : Math.signum(headingError);

        DistancePose2D normalizedError = new DistancePose2D(
                normVec,
                Angles.rad(normHeading)
        );

        DistancePose2D desiredVelocity = normalizedError.times(kV);

        previousError = err;

        if (!(resP.getVector2D().getX().isNaN() || resP.getVector2D().getY().isNaN() || resP.getHeading().isNaN()))
            res = res.plus(resP);
        if (!(resD.getVector2D().getX().isNaN() || resD.getVector2D().getY().isNaN() || resD.getHeading().isNaN()))
            res = res.plus(resD);
        if (!(desiredVelocity.getVector2D().getX().isNaN() || desiredVelocity.getVector2D().getY().isNaN() || desiredVelocity.getHeading().isNaN()))
            res = res.plus(desiredVelocity);

        return res;
    }

    @Override
    public void reset() {
        previousError = zero;
    }
}
