import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureIdlePathConstantMachSegment } from '@fmgc/flightmanagement/vnav/segments/PureIdlePathConstantMachSegment';
import { PureIdlePathDecelerationSegment } from './PureIdlePathDecelerationSegment';
import { PureIdlePathConstantSpeedSegment } from './PureIdlePathConstantSpeedSegment';

export class IdlePathSegment extends ProfileSegment {
    constructor(context: NodeContext, constraints: ConstraintReader, toAltitude: Feet) {
        super();

        const { descentSpeedLimit, managedDescentSpeed, managedDescentSpeedMach } = context.observer.get();

        const crossoverAltitude = context.computeCrossoverAltitude(managedDescentSpeed, managedDescentSpeedMach);

        this.children = [
            new IdlePathToAltitudeSegment(context, constraints, Math.min(descentSpeedLimit.underAltitude, crossoverAltitude, toAltitude), descentSpeedLimit.speed),
            new IdlePathToAltitudeSegment(context, constraints, Math.min(crossoverAltitude, toAltitude), managedDescentSpeed),
            new PureIdlePathConstantMachSegment(context, toAltitude, -Infinity),
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
            new PureIdlePathConstantSpeedSegment(this.context, this.toAltitude, -Infinity),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                continue;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureIdlePathConstantSpeedSegment(this.context, this.toAltitude, speedConstraint.distanceFromStart),
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
