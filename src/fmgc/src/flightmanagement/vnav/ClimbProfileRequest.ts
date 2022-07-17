import { IntegrationPropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { MaxAltitudeConstraint, MaxSpeedConstraint } from '@fmgc/guidance/vnav/ConstraintReader';
import { SpeedLimit } from '@fmgc/guidance/vnav/SpeedLimit';

export class ClimbProfileRequest {
    speedConstraints: MaxSpeedConstraint[] = []

    altitudeConstraints: MaxAltitudeConstraint[] = []

    speedLimit?: SpeedLimit = null;

    maxSpeed: Knots;

    maxMach: Mach;

    climbPropagator: IntegrationPropagator;

    accelerationPropagator: IntegrationPropagator;

    withSpeedConstraints(speedConstraints: MaxSpeedConstraint[]): ClimbProfileRequest {
        this.speedConstraints = speedConstraints;

        return this;
    }

    withAltitudeConstraints(altitudeConstraints: MaxAltitudeConstraint[]) {
        this.altitudeConstraints = altitudeConstraints;

        return this;
    }

    withSpeedLimit(speedLimit: SpeedLimit) {
        this.speedLimit = speedLimit;

        return this;
    }

    withMaxSpeed(maxSpeed: Knots) {
        this.maxSpeed = maxSpeed;

        return this;
    }

    withMaxMach(maxMach: Mach) {
        this.maxMach = maxMach;

        return this;
    }

    withClimbPropagator(climbPropagator: IntegrationPropagator) {
        this.climbPropagator = climbPropagator;

        return this;
    }

    withAccelerationPropagator(accelerationPropagator: IntegrationPropagator) {
        this.accelerationPropagator = accelerationPropagator;

        return this;
    }
}
