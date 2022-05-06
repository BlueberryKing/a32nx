import { AircraftConfiguration } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { NodeContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ConfigurationChangeSegment extends ProfileSegment {
    constructor(_context: NodeContext, private config: AircraftConfiguration) {
        super();
    }

    override compute(state: AircraftState, builder: ProfileBuilder): void {
        builder.push({
            ...state,
            config: this.config,
        });
    }

    get repr(): string {
        return `ConfigurationChangeNode - Change to ${JSON.stringify(this.config)}`;
    }
}
