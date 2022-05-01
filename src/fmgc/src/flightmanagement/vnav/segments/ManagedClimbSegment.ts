import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ClimbThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ClimbToAltConstraintSegment } from './ClimbToAltConstraintSegment';
import { PureAccelerationSegment } from './PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from './PureClimbToAltitudeSegment';
import { ProfileSegment, NodeContext } from './index';

export class ManagedClimbSegment extends ProfileSegment {
    constructor(context: NodeContext, maxSpeed: Knots, toAltitude: Feet, constraints: ConstraintReader) {
        super();

        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);

        this.children.push(
            new PureClimbToAltitudeSegment(context, climbThrust, toAltitude, Infinity),
        );

        const allConstraints = [...constraints.climbAlitudeConstraints, ...constraints.climbSpeedConstraints];
        allConstraints.sort((a, b) => b.distanceFromStart - a.distanceFromStart);

        let currentMaxSpeed = maxSpeed;
        let currentMaxAltitude = toAltitude;
        for (const constraint of allConstraints) {
            if ('maxSpeed' in constraint) {
                this.children.push(new PureAccelerationSegment(context, climbThrust, currentMaxSpeed, currentMaxAltitude));
                currentMaxSpeed = Math.min(currentMaxSpeed, constraint.maxSpeed);
            } else {
                this.children.push(new ClimbToAltConstraintSegment(context, climbThrust, constraint));
                currentMaxAltitude = Math.min(currentMaxAltitude, constraint.maxAltitude);
            }
        }

        this.children.push(
            new PureAccelerationSegment(context, climbThrust, currentMaxSpeed, currentMaxAltitude),
        );

        this.children = this.children.reverse();
    }

    get repr() {
        return 'ManagedClimbNode';
    }
}
