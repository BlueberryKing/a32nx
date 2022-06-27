import { ClimbThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';

export class ManagedClimbMachSegment extends ProfileSegment {
    constructor(context: SegmentContext, private toAltitude: Feet, private maxMach: Mach) {
        super();

        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);

        this.children = [
            PureAccelerationSegment.toMach(context, climbThrust, maxMach, toAltitude),
            new PureClimbToAltitudeSegment(context, climbThrust, toAltitude, true),
        ];
    }

    get repr() {
        return 'ManagedClimbMachSegment';
    }
}
