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

    /**
     *
     * @param context
     * @param constraints
     * @param maxSpeed
     * @param toAltitude This is used to implement the speed limit.
     */
    constructor(private context: NodeContext, private constraints: ConstraintReader, private maxSpeed: Knots, private toAltitude: Feet) {
        super();
    }

    setLastConstraint(constraint: DescentAltitudeConstraint) {
        this.lastConstraint = constraint;
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        this.children = [];

        if (this.lastConstraint === null) {
            return;
        }

        const preferredFlightPathAngle = this.calculatePreferredFlightPathAngle(state);

        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.distanceFromStart > state.distanceFromStart || constraint.distanceFromStart < this.lastConstraint.distanceFromStart) {
                continue;
            }

            this.children.unshift(
                new DescentAltitudeConstraintSegment(this.context, this.constraints, constraint, preferredFlightPathAngle, this.maxSpeed, this.toAltitude),
            );
        }
    }

    calculatePreferredFlightPathAngle(state: AircraftState) {
        return MathUtils.RADIANS_TO_DEGREES
            * Math.atan2(state.altitude - this.lastConstraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - this.lastConstraint.distanceFromStart));
    }

    get repr(): string {
        return 'GeometricPathSegment';
    }
}
