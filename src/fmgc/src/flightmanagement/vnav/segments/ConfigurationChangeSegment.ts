import { SegmentContext, AircraftState, ProfileBuilder, AircraftConfiguration } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { McduPseudoWaypointType, NdPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

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
                builder.requestMcduPseudoWaypoint(McduPseudoWaypointType.Flap2, builder.lastState);
                builder.requestNdPseudoWaypoint(NdPseudoWaypointType.Flap2, builder.lastState);
            } else if (this.config.flapConfig === FlapConf.CLEAN) {
                builder.requestMcduPseudoWaypoint(McduPseudoWaypointType.Flap1, builder.lastState);
                builder.requestNdPseudoWaypoint(NdPseudoWaypointType.Flap1, builder.lastState);
            }
        }
    }

    get repr(): string {
        return `ConfigurationChangeSegment - Change to ${JSON.stringify(this.config)}`;
    }
}
