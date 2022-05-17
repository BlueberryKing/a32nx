import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ClimbThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ClimbToAltConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/ClimbToAltConstraintSegment';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';
import { NodeContext } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ManagedClimbSegment extends ProfileSegment {
    constructor(context: NodeContext, maxSpeed: Knots, toAltitude: Feet, constraints: ConstraintReader) {
        super();

        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);

        this.children = [
            new PureClimbToAltitudeSegment(context, climbThrust, toAltitude),
        ];

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
