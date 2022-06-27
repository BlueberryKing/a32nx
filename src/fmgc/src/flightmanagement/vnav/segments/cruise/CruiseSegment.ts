import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { PureCruiseToDistanceSegment } from './PureCruiseToDistanceSegment';
import { PureCruiseStepSegment } from './PureCruiseStepSegment';

export class CruiseSegment extends ProfileSegment {
    constructor(context: SegmentContext, private steps: StepCoordinator, private fromDistance: NauticalMiles, private toDistance: NauticalMiles) {
        super();

        let altitude: Feet = context.observer.get().cruiseAltitude;

        const stepsInCruise = this.steps.steps.filter(({ distanceFromStart }) => distanceFromStart > fromDistance && distanceFromStart < toDistance);
        for (const step of stepsInCruise) {
            this.children.push(new PureCruiseStepSegment(context, step, altitude, toDistance));
            this.children.push(new PureCruiseToDistanceSegment(context, toDistance, step.toAltitude));

            altitude = step.toAltitude;
        }

        this.children.push(
            new PureCruiseToDistanceSegment(context, toDistance, altitude),
        );
    }

    get repr(): string {
        return `CruiseSegment - Cruise from ${Math.round(this.fromDistance).toFixed(0)} ft to ${Math.round(this.toDistance).toFixed(0)} ft.`;
    }
}
