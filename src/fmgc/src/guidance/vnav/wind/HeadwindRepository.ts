import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { WindProfileType } from './WindProfile';

export class HeadwindRepository {
    constructor(
        private climbHeadwind: HeadwindProfile,
        private cruiseHeadwind: HeadwindProfile,
        private descentHeadwind: HeadwindProfile,
    ) {

    }

    getWindProfile(type: WindProfileType): HeadwindProfile {
        switch (type) {
        case WindProfileType.Climb:
            return this.climbHeadwind;
        case WindProfileType.Cruise:
            return this.cruiseHeadwind;
        case WindProfileType.Descent:
            return this.descentHeadwind;
        default:
            throw new Error('[FMS/VNAV] Invalid wind profile type');
        }
    }
}
