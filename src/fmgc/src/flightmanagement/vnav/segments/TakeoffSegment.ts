import { FlapConf } from '@fmgc/guidance/vnav/common';
import { ClimbThrustSetting, TakeoffThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ConfigurationChangeSegment } from '@fmgc/flightmanagement/vnav/segments/ConfigurationChangeSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';
import { SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FmgcFlightPhase } from '@shared/flightphase';

export class TakeoffSegment extends ProfileSegment {
    constructor(context: SegmentContext) {
        super();

        const { thrustReductionAltitude, accelerationAltitude } = context.observer.get();

        this.children = [
            new PureClimbToAltitudeSegment(context, new TakeoffThrustSetting(context.atmosphericConditions), thrustReductionAltitude),
            new PureClimbToAltitudeSegment(context, new ClimbThrustSetting(context.atmosphericConditions), accelerationAltitude),
            new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CLEAN, speedbrakesExtended: false, gearExtended: false }),
        ];
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Climb);
    }

    get repr() {
        return 'TakeoffNode';
    }
}
