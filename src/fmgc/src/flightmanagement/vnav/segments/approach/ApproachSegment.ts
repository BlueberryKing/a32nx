import { SegmentContext, ProfileBuilder, AircraftState } from '@fmgc/flightmanagement/vnav/segments';
import { ConfigurationChangeSegment } from '@fmgc/flightmanagement/vnav/segments/ConfigurationChangeSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { FinalApproachSegment } from '@fmgc/flightmanagement/vnav/segments/approach/FinalApproachSegment';
import { FmgcFlightPhase } from '@shared/flightphase';
import { McduPseudoWaypointType, NdPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { ApproachFlapSegment } from '@fmgc/flightmanagement/vnav/segments/approach/ApproachFlapSegment';
import { ApproachInitialDecelerationSegment } from '@fmgc/flightmanagement/vnav/segments/approach/ApproachInitialDecelerationSegment';

/**
 * This represents a path from the Missed Approach Point to the Decel point, slowing the aircraft from descent speed to Vapp.
 */
export class ApproachSegment extends ProfileSegment {
    constructor(context: SegmentContext, constraints: ConstraintReader) {
        super();

        const { cleanSpeed, slatRetractionSpeed, flapRetractionSpeed, approachSpeed, isFlaps3Landing } = context.observer.get();
        const options: PropagatorOptions = {
            stepSize: -5,
            windProfileType: WindProfileType.Descent,
        };

        this.children = [
            new FinalApproachSegment(context, options),
        ];

        if (!isFlaps3Landing) {
            this.children.push(new ApproachFlapSegment(context, constraints, (flapRetractionSpeed + approachSpeed) / 2, options));
            this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_3 }));
        }

        this.children.push(new ApproachFlapSegment(context, constraints, flapRetractionSpeed, options)); /* In flaps 3 */
        this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_2, gearExtended: false }));
        this.children.push(new ApproachFlapSegment(context, constraints, slatRetractionSpeed, options)); /* In flaps 2 */
        this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_1 }, true));
        this.children.push(new ApproachFlapSegment(context, constraints, cleanSpeed, options)); /* In flaps 1 */
        this.children.push(new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CLEAN }, true));
        this.children.push(new ApproachInitialDecelerationSegment(context, constraints, options)); /* In clean configuration */
    }

    compute(_state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Approach);
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        // We want the speedtarget before the decel point to be whatever speed we have computed the deceleration to approach speed (approach segment) to
        builder.lastState.speeds.speedTarget = builder.lastState.speeds.calibratedAirspeed;

        builder.requestMcduPseudoWaypoint(McduPseudoWaypointType.Decel, builder.lastState);
        builder.requestNdPseudoWaypoint(NdPseudoWaypointType.Decel, builder.lastState);
    }

    get repr(): string {
        return 'ApproachSegment';
    }
}
