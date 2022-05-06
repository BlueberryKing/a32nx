import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureIdlePathDecelerationSegment } from './PureIdlePathDecelerationSegment';
import { PureIdlePathConstantSpeedSegment } from './PureIdlePathConstantSpeedSegment';

export class IdlePathSegment extends ProfileSegment {
    constructor(context: NodeContext, constraints: ConstraintReader, toAltitude: Feet) {
        super();

        const { descentSpeedLimit, managedDescentSpeed } = context.observer.get();

        this.children = [
            new IdlePathToAltitudeSegment(context, constraints, descentSpeedLimit.underAltitude, descentSpeedLimit.speed),
            new IdlePathToAltitudeSegment(context, constraints, toAltitude, managedDescentSpeed),
        ];
    }

    get repr(): string {
        return 'IdlePathSegment';
    }
}

export class IdlePathToAltitudeSegment extends ProfileSegment {
    constructor(private context: NodeContext, private constraints: ConstraintReader, private toAltitude: Feet, private maxSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        let maxSpeed = this.maxSpeed;

        this.children = [
            new PureIdlePathDecelerationSegment(this.context, this.toAltitude, maxSpeed, -Infinity),
            new PureIdlePathConstantSpeedSegment(this.context, this.toAltitude, maxSpeed, -Infinity),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                continue;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureIdlePathConstantSpeedSegment(this.context, this.toAltitude, maxSpeed, speedConstraint.distanceFromStart),
            );

            this.children.unshift(
                new PureIdlePathDecelerationSegment(this.context, this.toAltitude, maxSpeed, speedConstraint.distanceFromStart),
            );
        }
    }

    get repr(): string {
        return 'IdlePathToAltitudeSegment';
    }
}
