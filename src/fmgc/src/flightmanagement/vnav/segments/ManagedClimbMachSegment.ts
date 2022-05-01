import { ProfileSegment } from './index';

export class ManagedClimbMachSegment extends ProfileSegment {
    constructor(private toAltitude: Feet, private maxSpeed: Mach) {
        super();
    }

    get repr() {
        return 'ManagedClimbMachNode';
    }
}
