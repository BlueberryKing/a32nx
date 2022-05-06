import { PureConstantFlightPathAngleSegment } from '@fmgc/flightmanagement/vnav/segments/PureConstantFlightPathAngleSegment';
import { PureGeometricDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureGeometricDecelerationSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { DescentAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class DescentAltitudeConstraintSegment extends ProfileSegment {
    constructor(private context: NodeContext, private constraints: ConstraintReader, private constraint: DescentAltitudeConstraint, private flightPathAngle: Degrees, private maxSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        this.children = [
            new PureGeometricDecelerationSegment(this.context, this.flightPathAngle, this.maxSpeed, this.constraint.distanceFromStart),
            new PureConstantFlightPathAngleSegment(this.context, this.flightPathAngle, this.constraint.distanceFromStart),
        ];

        let maxSpeed = this.maxSpeed;

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);
            if (speedConstraint.distanceFromStart <= this.constraint.distanceFromStart || speedConstraint.distanceFromStart > state.distanceFromStart) {
                continue;
            }

            this.children.push(
                new PureConstantFlightPathAngleSegment(this.context, this.flightPathAngle, speedConstraint.distanceFromStart),
            );

            this.children.push(
                new PureGeometricDecelerationSegment(this.context, this.flightPathAngle, this.maxSpeed, speedConstraint.distanceFromStart),
            );
        }

        this.children.reverse();
    }

    get repr(): string {
        return `DescentAltitudeConstraintSegment - Descend at ${this.flightPathAngle.toFixed(2)}°`;
    }
}
