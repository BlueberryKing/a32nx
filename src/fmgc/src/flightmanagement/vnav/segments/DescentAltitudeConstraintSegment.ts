import { PureConstantFlightPathAngleSegment } from '@fmgc/flightmanagement/vnav/segments/PureConstantFlightPathAngleSegment';
import { PureGeometricDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureGeometricDecelerationSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class DescentAltitudeConstraintSegment extends ProfileSegment {
    constructor(
        private context: NodeContext,
        private constraints: ConstraintReader,
        private toDistance: NauticalMiles,
        private flightPathAngle: Degrees,
        private maxSpeed: Knots,
        private maxAltitude: Feet,
        private useMachVsCas: boolean = false,
    ) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        // If we're already above the `maxAltitude`, we can just return
        if (state.altitude >= this.maxAltitude) {
            return;
        }

        let maxSpeed = this.computeMaxSpeedOnSegment();

        this.children = [
            new PureGeometricDecelerationSegment(this.context, this.flightPathAngle, maxSpeed, this.toDistance, this.maxAltitude),
            new PureConstantFlightPathAngleSegment(this.context, this.flightPathAngle, this.toDistance, this.maxAltitude, this.useMachVsCas),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart <= this.toDistance) {
                continue;
            } else if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                break;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureConstantFlightPathAngleSegment(this.context, this.flightPathAngle, speedConstraint.distanceFromStart, this.maxAltitude, this.useMachVsCas),
            );

            this.children.unshift(
                new PureGeometricDecelerationSegment(this.context, this.flightPathAngle, maxSpeed, speedConstraint.distanceFromStart, this.maxAltitude),
            );
        }
    }

    get repr(): string {
        return `DescentAltitudeConstraintSegment - Descend at ${this.flightPathAngle.toFixed(2)}°`;
    }

    private computeMaxSpeedOnSegment(): Knots {
        // Use constraints before this segment to find the actual maxSpeed in this segment.
        return this.constraints.descentSpeedConstraints.reduce(
            (previous, current) => (current.distanceFromStart <= this.toDistance ? Math.min(previous, current.maxSpeed) : previous), this.maxSpeed,
        );
    }
}
