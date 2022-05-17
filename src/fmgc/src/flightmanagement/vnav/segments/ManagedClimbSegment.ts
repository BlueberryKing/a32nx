import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ClimbThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ClimbToAltConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/ClimbToAltConstraintSegment';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ManagedClimbSegment extends ProfileSegment {
    private climbThrust: ClimbThrustSetting;

    constructor(private context: NodeContext, private maxSpeed: Knots, private toAltitude: Feet, private constraints: ConstraintReader) {
        super();

        this.climbThrust = new ClimbThrustSetting(context.atmosphericConditions);

        this.children = [
            new PureClimbToAltitudeSegment(context, this.climbThrust, toAltitude),
        ];

        const allConstraints = [...constraints.climbAlitudeConstraints, ...constraints.climbSpeedConstraints];
        allConstraints.sort((a, b) => b.distanceFromStart - a.distanceFromStart);

        let currentMaxSpeed = maxSpeed;
        let currentMaxAltitude = toAltitude;
        for (const constraint of allConstraints) {
            if ('maxSpeed' in constraint) {
                // Is speed constraint
                this.children.push(new PureAccelerationSegment(context, this.climbThrust, currentMaxSpeed, currentMaxAltitude));
                currentMaxSpeed = Math.min(currentMaxSpeed, constraint.maxSpeed);
            } else if (constraint.maxAltitude <= toAltitude) {
                this.children.push(new ClimbToAltConstraintSegment(context, this.climbThrust, constraint));
                currentMaxAltitude = Math.min(currentMaxAltitude, constraint.maxAltitude);
            }
        }

        this.children = this.children.reverse();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        const maxSpeed = this.computeInitialMaxSpeed(state);
        const maxAltitude = this.computeInitialMaxAltitude(state);

        this.children.unshift(
            new PureAccelerationSegment(this.context, this.climbThrust, maxSpeed, maxAltitude),
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
