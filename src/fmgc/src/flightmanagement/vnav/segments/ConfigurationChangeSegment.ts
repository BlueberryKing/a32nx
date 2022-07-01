import { AircraftConfiguration } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

export class ConfigurationChangeSegment extends ProfileSegment {
    constructor(_context: SegmentContext, private config: Partial<AircraftConfiguration>, private emitFlapPseudoWaypoint: boolean = false) {
        super();
    }

    override compute(state: AircraftState, builder: ProfileBuilder): void {
        builder.push({
            ...state,
            config: { ...state.config, ...this.config },
        });

        if (this.emitFlapPseudoWaypoint) {
            if (this.config.flapConfig === FlapConf.CONF_1) {
                builder.requestPseudoWaypoint(McduPseudoWaypointType.Flap2, builder.lastState);
            } else if (this.config.flapConfig === FlapConf.CLEAN) {
                builder.requestPseudoWaypoint(McduPseudoWaypointType.Flap1, builder.lastState);
            }
        }
    }

    get repr(): string {
        return `ConfigurationChangeSegment - Change to ${JSON.stringify(this.config)}`;
    }
}
