import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureConstantFlightPathAngleSegment } from '@fmgc/flightmanagement/vnav/segments/PureConstantFlightPathAngleSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ApproachAltitudeConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/ApproachAltitudeConstraintSegment';
import { PureApproachDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureApproachDecelerationSegment';

export class ApproachFlapSegment extends ProfileSegment {
    constructor(private context: NodeContext, private constraints: ConstraintReader, private nextFlapSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        // Why this? This is about -500 fpm at 200 kts, which should achieve decent deceleration
        // TODO: Use better method for this.
        const preferredFlightPathAngle: Degrees = -1.5;
        let maxSpeed = this.nextFlapSpeed;

        this.children = [
            new PureApproachDecelerationSegment(
                this.context,
                preferredFlightPathAngle,
                this.nextFlapSpeed,
                -Infinity,
            ),
        ];

        // Speed constraints appearing before any altitude constraints, which inhibit a direct deceleration to the flap extension speed
        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            // If the constraint is "behind" the aircraft in the reverse profile, we ignore it
            // Also, if the constraint is between two altitude constraints, it will be treated further down.
            if (state.distanceFromStart <= speedConstraint.distanceFromStart
                || this.constraints.descentAltitudeConstraints.length > 0 && speedConstraint.distanceFromStart >= this.constraints.descentAltitudeConstraints[0].distanceFromStart) {
                break;
                // If the constraint does not actually stop us from "accelerating" to the flap extension speed, ignore it
            } else if (speedConstraint.maxSpeed > this.nextFlapSpeed) {
                continue;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureApproachDecelerationSegment(this.context, preferredFlightPathAngle, maxSpeed, speedConstraint.distanceFromStart),
            );

            this.children.unshift(
                new PureConstantFlightPathAngleSegment(this.context, preferredFlightPathAngle, speedConstraint.distanceFromStart),
            );
        }

        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.distanceFromStart > state.distanceFromStart) {
                continue;
            }

            this.children.unshift(
                new ApproachAltitudeConstraintSegment(this.context, this.constraints, constraint, preferredFlightPathAngle, maxSpeed),
            );
        }
    }

    get repr(): string {
        return 'ApproachFlapSegment';
    }
}
