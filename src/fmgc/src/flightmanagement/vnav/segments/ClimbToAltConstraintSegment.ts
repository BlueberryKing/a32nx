import { MaxAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { PureLevelSegment } from '@fmgc/flightmanagement/vnav/segments/PureLevelSegment';
import { NodeContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';

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
