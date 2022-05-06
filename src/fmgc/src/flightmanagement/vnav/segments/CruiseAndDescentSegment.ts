import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class CruiseAndDescentSegment extends ProfileSegment {
    get repr() {
        return 'CruiseAndDescentNode';
    }
}
