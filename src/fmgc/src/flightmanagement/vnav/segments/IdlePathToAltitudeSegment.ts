import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { PureIdlePathDecelerationSegment } from './PureIdlePathDecelerationSegment';
import { PureIdlePathConstantSpeedSegment } from './PureIdlePathConstantSpeedSegment';

export class IdlePathToAltitudeSegment extends ProfileSegment {
    constructor(private context: SegmentContext, private constraints: ConstraintReader, private toAltitude: Feet, private maxSpeed: Knots, private options: PropagatorOptions) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        let maxSpeed = this.maxSpeed;

        this.children = [
            new PureIdlePathDecelerationSegment(this.context, this.toAltitude, maxSpeed, -Infinity, this.options),
            new PureIdlePathConstantSpeedSegment(this.context, this.toAltitude, -Infinity, this.options),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                continue;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureIdlePathConstantSpeedSegment(this.context, this.toAltitude, speedConstraint.distanceFromStart, this.options),
            );

            this.children.unshift(
                new PureIdlePathDecelerationSegment(this.context, this.toAltitude, maxSpeed, speedConstraint.distanceFromStart, this.options),
            );
        }
    }

    get repr(): string {
        return 'IdlePathToAltitudeSegment';
    }
}
