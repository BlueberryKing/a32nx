import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ApproachSegment extends ProfileSegment {
    get repr(): string {
        return 'ApproachSegment';
    }
}
