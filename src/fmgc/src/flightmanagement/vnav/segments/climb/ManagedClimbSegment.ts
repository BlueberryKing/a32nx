import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ClimbThrustSetting, PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureClimbToAltitudeSegment';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureLevelAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureLevelAccelerationSegment';
import { PureLevelSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureLevelSegment';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { ClimbToAltConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ClimbToAltConstraintSegment';

export class ManagedClimbSegment extends ProfileSegment {
    private climbThrust: ClimbThrustSetting;

    private options: PropagatorOptions = { useMachVsCas: false, stepSize: 5, windProfileType: WindProfileType.Climb };

    constructor(private context: SegmentContext, private maxSpeed: Knots, private toAltitude: Feet, private constraints: ConstraintReader) {
        super();

        this.climbThrust = new ClimbThrustSetting(context.atmosphericConditions);

        this.children = [
            new PureClimbToAltitudeSegment(context, this.climbThrust, toAltitude, this.options),
        ];

        const allConstraints = [...constraints.climbSpeedConstraints, ...constraints.climbAlitudeConstraints];
        allConstraints.sort((a, b) => b.distanceFromStart - a.distanceFromStart);

        let currentMaxSpeed = maxSpeed;
        let currentMaxAltitude = toAltitude;
        for (const constraint of allConstraints) {
            // Is speed constraint
            // We only consider speed constraints if they are more restrictive than a later speed constraint
            if ('maxSpeed' in constraint) {
                // If this constraint is not more restrictive than a later one, we can ignore it.
                if (constraint.maxSpeed >= currentMaxSpeed) {
                    continue;
                }

                // We only need to do this if we're below `toAltitude` because if we're above it, we can let the next `ManagedClimbSegment` take care of them
                if (currentMaxAltitude < toAltitude) {
                    // The following two segments make sure we get to the next constraint after dealing with `constraint`.
                    // Accelerate in level flight if an upcoming constraint requires it.
                    this.children.push(new PureLevelAccelerationSegment(context, this.climbThrust, currentMaxSpeed, Infinity, this.options));
                    // Accelerate after `constraint`
                    this.children.push(new PureAccelerationSegment(context, this.climbThrust, currentMaxSpeed, currentMaxAltitude, this.options));

                    // Possibly fly level before reaching `constraint` if there is a constraing altitude constraint
                    this.children.push(new PureLevelSegment(context, constraint.distanceFromStart, this.options));
                }

                // Fly to `constraint` first.
                this.children.push(new PureClimbToAltitudeSegment(context, this.climbThrust, currentMaxAltitude, this.options, constraint.distanceFromStart));

                currentMaxSpeed = Math.min(currentMaxSpeed, constraint.maxSpeed);
            } else if (constraint.maxAltitude <= toAltitude) {
                this.children.push(new ClimbToAltConstraintSegment(context, this.climbThrust, constraint.maxAltitude, constraint.distanceFromStart, this.options));
                currentMaxAltitude = Math.min(currentMaxAltitude, constraint.maxAltitude);
            }
        }

        this.children = this.children.reverse();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        const maxSpeed = this.computeInitialMaxSpeed(state);
        const maxAltitude = this.computeInitialMaxAltitude(state);

        this.children.unshift(
            new PureAccelerationSegment(this.context, this.climbThrust, maxSpeed, maxAltitude, this.options),
        );
    }

    private computeInitialMaxSpeed(state: AircraftState): Knots {
        return this.constraints.climbSpeedConstraints.reduce(
            (previous, current) => (state.distanceFromStart <= current.distanceFromStart ? Math.min(previous, current.maxSpeed) : previous), this.maxSpeed,
        );
    }

    private computeInitialMaxAltitude(state: AircraftState): Knots {
        return this.constraints.climbAlitudeConstraints.reduce(
            (previous, current) => (state.distanceFromStart <= current.distanceFromStart ? Math.min(previous, current.maxAltitude) : previous), this.toAltitude,
        );
    }

    get repr() {
        return 'ManagedClimbSegment';
    }
}
