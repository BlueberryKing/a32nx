import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

/**
 * Represents segment from 1000 ft above the threshold to the Missed Approach Point
 */

export class StabilizedFinalApproachSegment extends ProfileSegment {
    get repr(): string {
        return 'StabilizedFinalApproachSegment';
    }
}
