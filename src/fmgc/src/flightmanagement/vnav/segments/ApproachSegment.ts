import { SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ConfigurationChangeSegment } from '@fmgc/flightmanagement/vnav/segments/ConfigurationChangeSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ApproachFlapSegment } from '@fmgc/flightmanagement/vnav/segments/ApproachFlapSegment';
import { FinalApproachSegment } from '@fmgc/flightmanagement/vnav/segments/FinalApproachSegment';
import { FmgcFlightPhase } from '@shared/flightphase';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { ApproachInitialDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/ApproachInitialDecelerationSegment';

/**
 * This represents a path from the Missed Approach Point to the Decel point, slowing the aircraft from descent speed to Vapp.
 */
export class ApproachSegment extends ProfileSegment {
    constructor(context: SegmentContext, constraints: ConstraintReader) {
        super();

        const { cleanSpeed, slatRetractionSpeed, flapRetractionSpeed, approachSpeed, isFlaps3Landing } = context.observer.get();

        this.children = [
            new FinalApproachSegment(context),
        ];

        if (!isFlaps3Landing) {
            this.children.push(new ApproachFlapSegment(context, constraints, (flapRetractionSpeed + approachSpeed) / 2));
            this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_3 }));
        }

        this.children.push(new ApproachFlapSegment(context, constraints, flapRetractionSpeed)); /* In flaps 3 */
        this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_2, gearExtended: false }));
        this.children.push(new ApproachFlapSegment(context, constraints, slatRetractionSpeed)); /* In flaps 2 */
        this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_1 }, true));
        this.children.push(new ApproachFlapSegment(context, constraints, cleanSpeed)); /* In flaps 1 */
        this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CLEAN }, true));
        this.children.push(new ApproachInitialDecelerationSegment(context, constraints)); /* In clean configuration */
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.requestPseudoWaypoint(McduPseudoWaypointType.Decel, builder.lastState);
        builder.changePhase(FmgcFlightPhase.Descent);
    }

    get repr(): string {
        return 'ApproachSegment';
    }
}
