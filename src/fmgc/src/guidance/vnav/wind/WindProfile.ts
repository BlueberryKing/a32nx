import { WindComponent } from '@fmgc/guidance/vnav/wind';

export enum WindProfileType {
    Climb,
    Cruise,
    Descent,
}

export interface WindProfile {
    getHeadwindComponent(distanceFromStart: NauticalMiles, altitude: Feet, planeHeading: DegreesTrue): WindComponent
}
