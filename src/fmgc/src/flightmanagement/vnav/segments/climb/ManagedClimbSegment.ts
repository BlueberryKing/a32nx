import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ClimbThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureClimbToAltitudeSegment';
import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureLevelAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureLevelAccelerationSegment';
import { PureLevelSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureLevelSegment';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';

export class ManagedClimbSegment extends ProfileSegment {
    constructor(context: SegmentContext, private maxSpeed: Knots, private toAltitude: Feet, private constraints: ConstraintReader) {
        super();

        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);
        const options = { stepSize: 5, windProfileType: WindProfileType.Climb };

        this.children = [
            new PureClimbToAltitudeSegment(context, climbThrust, toAltitude, options),
            new PureAccelerationSegment(context, climbThrust, maxSpeed, toAltitude, options),
        ];

        const allConstraints = [...constraints.climbSpeedConstraints, ...constraints.climbAlitudeConstraints];
        allConstraints.sort((a, b) => b.distanceFromStart - a.distanceFromStart);

        let currentMaxSpeed = maxSpeed;
        let currentMaxAltitude = toAltitude;
        for (const constraint of allConstraints) {
            // Check if constraint is speed constraint
            if ('maxSpeed' in constraint) {
                // If this constraint is not more restrictive than a later one, we can ignore it.
                if (constraint.maxSpeed >= currentMaxSpeed) {
                    continue;
                }

                // We only need to do this if we're below `toAltitude` because if we're above it, we can let the next `ManagedClimbSegment` take care of them
                if (currentMaxAltitude < toAltitude) {
                    // Possibly fly level before reaching `constraint` if there is a constraining altitude constraint
                    this.children.push(new PureLevelSegment(context, constraint.distanceFromStart, options));
                    // Accelerate in level flight if an upcoming constraint requires it.
                    this.children.push(new PureLevelAccelerationSegment(context, climbThrust, constraint.maxSpeed, constraint.distanceFromStart, options));
                }

                this.children.push(new PureClimbToAltitudeSegment(context, climbThrust, currentMaxAltitude, options, constraint.distanceFromStart));
                this.children.push(new PureAccelerationSegment(context, climbThrust, constraint.maxSpeed, currentMaxAltitude, options, constraint.distanceFromStart));

                currentMaxSpeed = Math.min(currentMaxSpeed, constraint.maxSpeed);
            } else if (constraint.maxAltitude <= toAltitude) {
                this.children.push(new PureLevelSegment(context, constraint.distanceFromStart, options));
                this.children.push(new PureClimbToAltitudeSegment(context, climbThrust, constraint.maxAltitude, options, constraint.distanceFromStart));

                // Before climbing to the constraint, accelerate to the target speed
                this.children.push(new PureLevelAccelerationSegment(context, climbThrust, currentMaxSpeed, constraint.distanceFromStart, options));
                this.children.push(new PureAccelerationSegment(context, climbThrust, currentMaxSpeed, constraint.maxAltitude, options));

                currentMaxAltitude = Math.min(currentMaxAltitude, constraint.maxAltitude);
            }
        }

        this.children = this.children.reverse();
    }

    get repr() {
        return 'ManagedClimbSegment';
    }
}
