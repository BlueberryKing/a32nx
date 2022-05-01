import { MaxAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { PureLevelSegment } from './PureLevelSegment';
import { ProfileSegment, NodeContext } from './index';
import { PureClimbToAltitudeSegment } from './PureClimbToAltitudeSegment';

export class ClimbToAltConstraintSegment extends ProfileSegment {
    constructor(context: NodeContext, thrustSetting: ThrustSetting, private constraint: MaxAltitudeConstraint) {
        super();

        this.children = [
            new PureClimbToAltitudeSegment(context, thrustSetting, constraint.maxAltitude, constraint.distanceFromStart),
            new PureLevelSegment(context, constraint.distanceFromStart),
        ];
    }

    get repr() {
        return `ClimbToAltConstraint - Stay below ${this.constraint.maxAltitude} ft`;
    }
}
