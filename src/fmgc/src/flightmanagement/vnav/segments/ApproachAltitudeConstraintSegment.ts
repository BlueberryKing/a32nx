import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureConstantFlightPathAngleSegment } from '@fmgc/flightmanagement/vnav/segments/PureConstantFlightPathAngleSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { ConstraintReader, DescentAltitudeConstraint } from '@fmgc/guidance/vnav/ConstraintReader';
import { MathUtils } from '@shared/MathUtils';
import { PureApproachDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureApproachDecelerationSegment';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';

export class ApproachAltitudeConstraintSegment extends ProfileSegment {
    constructor(
        private context: SegmentContext,
        private constraints: ConstraintReader,
        private constraint: DescentAltitudeConstraint,
        private preferredFlightPathAngle: Degrees,
        private maxSpeed: Knots,
        private options: PropagatorOptions,
    ) {
        super();
    }

    get repr(): string {
        return 'ApproachAltitudeConstraintSegment';
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        // TODO: Try get FPA from previous segment to minimize pitch changes.
        const [minAngle, maxAngle] = this.getFlightPathAngleRange(state, this.constraint);
        const flightPathAngle = Math.max(minAngle, Math.min(maxAngle, this.preferredFlightPathAngle));

        let maxSpeed = this.maxSpeed;

        this.children = [
            new PureApproachDecelerationSegment(this.context, flightPathAngle, maxSpeed, this.constraint.distanceFromStart, this.options),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart < this.constraint.distanceFromStart || speedConstraint.maxSpeed > this.maxSpeed) {
                continue;
            } else if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                break;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            // This makes sure to fly to the speed constraint, as we will need to pass it to all further "acceleration" to the flap speed.
            this.children.unshift(
                new PureConstantFlightPathAngleSegment(this.context, flightPathAngle, speedConstraint.distanceFromStart, this.options),
            );

            this.children.unshift(
                new PureApproachDecelerationSegment(this.context, flightPathAngle, maxSpeed, speedConstraint.distanceFromStart, this.options),
            );
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
