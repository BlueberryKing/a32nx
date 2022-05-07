import { PureConstantFlightPathAngleSegment } from '@fmgc/flightmanagement/vnav/segments/PureConstantFlightPathAngleSegment';
import { PureGeometricDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureGeometricDecelerationSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { DescentAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { MathUtils } from '@shared/MathUtils';
import { AltitudeConstraint, AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';

export class DescentAltitudeConstraintSegment extends ProfileSegment {
    constructor(private context: NodeContext, private constraints: ConstraintReader, private constraint: DescentAltitudeConstraint, private flightPathAngle: Degrees, private maxSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        const altAtConstraint = state.altitude + 6076.12 * (this.constraint.distanceFromStart - state.distanceFromStart) * Math.tan(this.flightPathAngle * MathUtils.DEGREES_TO_RADIANS);
        if (VnavConfig.DEBUG_PROFILE && !this.isAltitudeConstraintMet(altAtConstraint, this.constraint.constraint)) {
            console.warn(`[FMS/VNAV] Expecting to miss constraint. Predicting altitude ${altAtConstraint.toFixed(0)}, but should be at ${JSON.stringify(this.constraint.constraint)}`);
        }

        // Use constraints before this segment to find the actual maxSpeed in this segment.
        let maxSpeed = this.constraints.descentSpeedConstraints.reduce(
            (previous, current) => (current.distanceFromStart <= this.constraint.distanceFromStart ? Math.min(previous, current.maxSpeed) : previous), this.maxSpeed,
        );

        this.children = [
            new PureGeometricDecelerationSegment(this.context, this.flightPathAngle, maxSpeed, this.constraint.distanceFromStart),
            new PureConstantFlightPathAngleSegment(this.context, this.flightPathAngle, this.constraint.distanceFromStart),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart <= this.constraint.distanceFromStart) {
                continue;
            } else if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                break;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureConstantFlightPathAngleSegment(this.context, this.flightPathAngle, speedConstraint.distanceFromStart),
            );

            this.children.unshift(
                new PureGeometricDecelerationSegment(this.context, this.flightPathAngle, this.maxSpeed, speedConstraint.distanceFromStart),
            );
        }
    }

    private isAltitudeConstraintMet(altitude: Feet, constraint?: AltitudeConstraint): boolean {
        if (!constraint) {
            return true;
        }

        switch (constraint.type) {
        case AltitudeConstraintType.at:
            return Math.abs(altitude - constraint.altitude1) < 250;
        case AltitudeConstraintType.atOrAbove:
            return (altitude - constraint.altitude1) > -250;
        case AltitudeConstraintType.atOrBelow:
            return (altitude - constraint.altitude1) < 250;
        case AltitudeConstraintType.range:
            return (altitude - constraint.altitude2) > -250 && (altitude - constraint.altitude1) < 250;
        default:
            console.error('Invalid altitude constraint type');
            return null;
        }
    }

    get repr(): string {
        return `DescentAltitudeConstraintSegment - Descend at ${this.flightPathAngle.toFixed(2)}°`;
    }
}
