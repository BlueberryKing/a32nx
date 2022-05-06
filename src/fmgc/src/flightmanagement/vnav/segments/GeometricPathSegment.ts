import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { DescentAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { MathUtils } from '@shared/MathUtils';
import { DescentAltitudeConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/DescentAltitudeConstraintSegment';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class GeometricPathSegment extends ProfileSegment {
    /**
     * The `lastConstraint` is the first constraint encountered along the lateral track.
     */
    private lastConstraint?: DescentAltitudeConstraint = null;

    constructor(private context: NodeContext, private constraints: ConstraintReader, private maxSpeed: Knots) {
        super();
    }

    setLastConstraint(constraint: DescentAltitudeConstraint) {
        this.lastConstraint = constraint;
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        if (this.lastConstraint === null) {
            return;
        }

        const preferredFlightPathAngle = this.calculatePreferredFlightPathAngle(state);

        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.distanceFromStart > state.distanceFromStart || constraint.distanceFromStart < this.lastConstraint.distanceFromStart) {
                continue;
            }

            this.children.push(
                new DescentAltitudeConstraintSegment(this.context, this.constraints, constraint, preferredFlightPathAngle, this.maxSpeed),
            );
        }

        this.children.reverse();
    }

    calculatePreferredFlightPathAngle(state: AircraftState) {
        return MathUtils.RADIANS_TO_DEGREES
            * Math.atan2(this.lastConstraint.constraint.altitude1 - state.altitude, 6076.12 * (this.lastConstraint.distanceFromStart - state.distanceFromStart));
    }

    get repr(): string {
        return 'GeometricPathSegment';
    }
}
