import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ManagedClimbMachSegment extends ProfileSegment {
    constructor(private toAltitude: Feet, private maxSpeed: Mach) {
        super();
    }

    get repr() {
        return 'ManagedClimbMachNode';
    }
}
