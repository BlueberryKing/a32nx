import { FlapConf } from '@fmgc/guidance/vnav/common';
import { ClimbThrustSetting, PropagatorOptions, TakeoffThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ConfigurationChangeSegment } from '@fmgc/flightmanagement/vnav/segments/ConfigurationChangeSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';
import { SegmentContext, ProfileBuilder, AircraftState } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FmgcFlightPhase } from '@shared/flightphase';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';

export class TakeoffSegment extends ProfileSegment {
    constructor(private context: SegmentContext) {
        super();

        const { thrustReductionAltitude, accelerationAltitude } = context.observer.get();
        const options: PropagatorOptions = { useMachVsCas: false, stepSize: 5, windProfileType: WindProfileType.Climb };

        this.children = [
            new PureClimbToAltitudeSegment(context, new TakeoffThrustSetting(context.atmosphericConditions), thrustReductionAltitude, options),
            new PureClimbToAltitudeSegment(context, new ClimbThrustSetting(context.atmosphericConditions), accelerationAltitude, options),
            new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CLEAN, speedbrakesExtended: false, gearExtended: false }),
        ];
    }

    shouldCompute(_state: AircraftState, _builder: ProfileBuilder): boolean {
        return this.context.observer.get().flightPhase <= FmgcFlightPhase.Takeoff;
    }

    compute(_state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Takeoff);
    }

    get repr() {
        return 'TakeoffNode';
    }
}
