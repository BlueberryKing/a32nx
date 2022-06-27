import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { ConstraintReader, DescentAltitudeConstraint } from '@fmgc/guidance/vnav/ConstraintReader';
import { MathUtils } from '@shared/MathUtils';
import { PureInitialApproachDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureInitialApproachDecelerationSegment';
import { SpeedLimit } from '@fmgc/guidance/vnav/SpeedLimit';
import { PureApproachDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureApproachDecelerationSegment';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';

export class InitialApproachAltitudeConstraintSegment extends ProfileSegment {
    private managedDescentSpeed: Knots;

    private descentSpeedLimit: SpeedLimit;

    constructor(
        private context: SegmentContext,
        private constraints: ConstraintReader,
        private constraint: DescentAltitudeConstraint,
        private preferredFlightPathAngle: Degrees,
        private options: PropagatorOptions,
    ) {
        super();

        const { managedDescentSpeed, descentSpeedLimit } = context.observer.get();

        this.managedDescentSpeed = managedDescentSpeed;
        this.descentSpeedLimit = descentSpeedLimit;
    }

    get repr(): string {
        return 'InitialApproachAltitudeConstraintSegment';
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        this.children = [];

        // TODO: Try get FPA from previous segment to minimize pitch changes.
        const [minAngle, maxAngle] = this.getFlightPathAngleRange(state, this.constraint);
        const flightPathAngle = Math.max(minAngle, Math.min(maxAngle, this.preferredFlightPathAngle));

        let maxSpeed = this.descentSpeedWithSpeedLimit(state.altitude);
        let distanceToLastConstraint = this.constraint.distanceFromStart;

        for (const speedConstraint of this.constrainingSpeedConstraints(state)) {
            if (speedConstraint.distanceFromStart < this.constraint.distanceFromStart || speedConstraint.maxSpeed > maxSpeed) {
                maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

                continue;
            }

            this.children.unshift(
                new PureInitialApproachDecelerationSegment(this.context, flightPathAngle, maxSpeed, speedConstraint.distanceFromStart, this.options),
            );

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);
            distanceToLastConstraint = Math.max(distanceToLastConstraint, speedConstraint.distanceFromStart);
        }

        this.children.unshift(
            new PureApproachDecelerationSegment(this.context, flightPathAngle, maxSpeed, distanceToLastConstraint, this.options),
        );
    }

    private descentSpeedWithSpeedLimit(altitude: Feet): Knots {
        let maxSpeed = this.managedDescentSpeed;
        if (Number.isFinite(this.descentSpeedLimit.speed) && Number.isFinite(this.descentSpeedLimit.underAltitude) && altitude < this.descentSpeedLimit.underAltitude) {
            maxSpeed = Math.min(maxSpeed, this.descentSpeedLimit.speed);
        }

        return maxSpeed;
    }

    private* constrainingSpeedConstraints(state: AircraftState) {
        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                break;
            }

            yield speedConstraint;
        }
    }

    private getFlightPathAngleRange(state: AircraftState, constraint: DescentAltitudeConstraint): [Degrees, Degrees] {
        switch (constraint.constraint.type) {
        case AltitudeConstraintType.at:
            return [
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
            ];
        case AltitudeConstraintType.atOrAbove:
            return [
                -20,
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
            ];
        case AltitudeConstraintType.atOrBelow:
            return [
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
                0,
            ];
        case AltitudeConstraintType.range:
            return [
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude2, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
            ];
        default:
            console.error('Invalid altitude constraint type');
            return null;
        }
    }
}
