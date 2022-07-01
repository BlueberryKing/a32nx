import { ClimbThrustSetting, PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureClimbToAltitudeSegment';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';

export class ManagedClimbMachSegment extends ProfileSegment {
    constructor(context: SegmentContext, private toAltitude: Feet, private maxMach: Mach) {
        super();

        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);
        const options: PropagatorOptions = { useMachVsCas: true, stepSize: 5, windProfileType: WindProfileType.Climb };

        this.children = [
            PureAccelerationSegment.toMach(context, climbThrust, maxMach, toAltitude, options),
            new PureClimbToAltitudeSegment(context, climbThrust, toAltitude, options),
        ];
    }

    get repr() {
        return 'ManagedClimbMachSegment';
    }
}
