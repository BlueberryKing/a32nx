import { FlapConf } from '@fmgc/guidance/vnav/common';
import { ClimbThrustSetting, TakeoffThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ConfigurationChangeSegment } from './ConfigurationChangeSegment';
import { PureClimbToAltitudeSegment } from './PureClimbToAltitudeSegment';
import { ProfileSegment, NodeContext } from './index';

export class TakeoffSegment extends ProfileSegment {
    constructor(context: NodeContext) {
        super();

        const { thrustReductionAltitude, accelerationAltitude } = context.observer.get();

        this.children = [
            new PureClimbToAltitudeSegment(context, new TakeoffThrustSetting(context.atmosphericConditions), thrustReductionAltitude),
            new PureClimbToAltitudeSegment(context, new ClimbThrustSetting(context.atmosphericConditions), accelerationAltitude),
            new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CLEAN, speedbrakesExtended: false, gearExtended: false }),
        ];
    }

    get repr() {
        return 'TakeoffNode';
    }
}
