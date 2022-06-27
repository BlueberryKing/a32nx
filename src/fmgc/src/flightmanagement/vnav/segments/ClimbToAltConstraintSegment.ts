import { ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { PureLevelSegment } from '@fmgc/flightmanagement/vnav/segments/PureLevelSegment';
import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/PureClimbToAltitudeSegment';

export class ClimbToAltConstraintSegment extends ProfileSegment {
    constructor(context: SegmentContext, thrustSetting: ThrustSetting, private toAltitude: Feet, private toDistance: NauticalMiles) {
        super();

        this.children = [
            new PureClimbToAltitudeSegment(context, thrustSetting, toAltitude, false, toDistance),
            new PureLevelSegment(context, toDistance),
        ];
    }

    get repr() {
        return `ClimbToAltConstraint - Stay below ${this.toAltitude} ft`;
    }
}
