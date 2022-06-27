import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { SpeedLimit } from '@fmgc/guidance/vnav/SpeedLimit';
import { PureInitialApproachDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureInitialApproachDecelerationSegment';
import { InitialApproachAltitudeConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/InitialApproachAltitudeConstraintSegment';

/**
 * This segment represents the deceleration from whatever the descent speed is to green dot speed.
 * It is special in that sense that we don't know what the "descent speed" will be, as it depends on the configuration of speed constraints.
 */
export class ApproachInitialDecelerationSegment extends ProfileSegment {
    private managedDescentSpeed: Knots;

    private descentSpeedLimit: SpeedLimit;

    constructor(private context: SegmentContext, private constraints: ConstraintReader) {
        super();

        const { managedDescentSpeed, descentSpeedLimit } = context.observer.get();

        this.managedDescentSpeed = managedDescentSpeed;
        this.descentSpeedLimit = descentSpeedLimit;
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        // Why this? This is about -500 fpm at 200 kts, which should achieve decent deceleration
        // TODO: Use better method for this.
        const preferredFlightPathAngle: Degrees = -1.5;

        // Initial estimation for the descent speed.
        let maxSpeed = this.descentSpeedWithSpeedLimit(state.altitude);

        this.children = [
            new PureInitialApproachDecelerationSegment(
                this.context,
                preferredFlightPathAngle,
                maxSpeed,
                this.constraints.descentSpeedConstraints.length > 0 ? this.constraints.descentSpeedConstraints[0].distanceFromStart : Infinity,
            ),
        ];

        // Handle speed constraints appearing before any altitude constraints
        for (const speedConstraint of this.speedConstraintsBeforeAltitudeConstraints(state)) {
            if (speedConstraint.maxSpeed > maxSpeed) {
                continue;
            }

            // Decelerate to the descent speed if we're past this constraint
            this.children.unshift(
                new PureInitialApproachDecelerationSegment(this.context, preferredFlightPathAngle, maxSpeed, speedConstraint.distanceFromStart),
            );

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);
        }

        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.distanceFromStart > state.distanceFromStart) {
                break;
            }

            this.children.unshift(
                new InitialApproachAltitudeConstraintSegment(this.context, this.constraints, constraint, preferredFlightPathAngle),
            );
        }
    }

    private descentSpeedWithSpeedLimit(altitude: Feet): Knots {
        let maxSpeed = this.managedDescentSpeed;
        if (Number.isFinite(this.descentSpeedLimit.speed) && Number.isFinite(this.descentSpeedLimit.underAltitude) && altitude < this.descentSpeedLimit.underAltitude) {
            maxSpeed = Math.min(maxSpeed, this.descentSpeedLimit.speed);
        }

        return maxSpeed;
    }

    private* speedConstraintsBeforeAltitudeConstraints(state: AircraftState) {
        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            // If the constraint is "behind" the aircraft in the reverse profile, we ignore it
            // Also if the constraint appears behind an altitude constraint, we ignore it since these are handled as part of InitialApproachAltitudeConstraintSegment
            if (state.distanceFromStart <= speedConstraint.distanceFromStart
                || this.constraints.descentAltitudeConstraints.length > 0 && speedConstraint.distanceFromStart >= this.constraints.descentAltitudeConstraints[0].distanceFromStart) {
                break;
                // If the constraint does not actually stop us from "accelerating" to the descent speed, ignore it
            }

            yield speedConstraint;
        }
    }

    get repr(): string {
        return 'ApproachInitialDecelerationSegment';
    }
}
