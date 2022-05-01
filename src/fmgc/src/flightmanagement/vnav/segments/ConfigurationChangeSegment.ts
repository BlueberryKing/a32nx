import { AircraftConfiguration } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { ProfileSegment, NodeContext, AircraftState, ProfileBuilder } from './index';

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
