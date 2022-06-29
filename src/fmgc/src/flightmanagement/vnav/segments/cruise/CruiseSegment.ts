import { AircraftState, ProfileBuilder, SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { FmgcFlightPhase } from '@shared/flightphase';
import { PureCruiseToDistanceSegment } from './PureCruiseToDistanceSegment';
import { PureCruiseStepSegment } from './PureCruiseStepSegment';

export class CruiseSegment extends ProfileSegment {
    constructor(private context: SegmentContext, private steps: StepCoordinator, private fromDistance: NauticalMiles, private toDistance: NauticalMiles) {
        super();

        const { cruiseAltitude } = context.observer.get();

        // There is some weird logic with how it determines whether to use mach or cas for steps
        // This logic should be tackled in the step itself.
        const options = {
            stepSize: 5,
            windProfileType: WindProfileType.Cruise,
        };

        let altitude = cruiseAltitude;

        const stepsInCruise = this.steps.steps.filter(({ distanceFromStart }) => distanceFromStart > fromDistance && distanceFromStart < toDistance);
        for (const step of stepsInCruise) {
            this.children.push(new PureCruiseStepSegment(context, step, altitude, toDistance, options));
            this.children.push(new PureCruiseToDistanceSegment(context, toDistance, step.toAltitude, options));

            altitude = step.toAltitude;
        }

        this.children.push(
            new PureCruiseToDistanceSegment(context, toDistance, altitude, options),
        );
    }

    shouldCompute(_state: AircraftState, _builder: ProfileBuilder): boolean {
        return this.context.observer.get().flightPhase <= FmgcFlightPhase.Cruise;
    }

    compute(_state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Cruise);
    }

    get repr(): string {
        return `CruiseSegment - Cruise from ${Math.round(this.fromDistance).toFixed(0)} ft to ${Math.round(this.toDistance).toFixed(0)} ft.`;
    }
}
