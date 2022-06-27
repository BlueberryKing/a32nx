import {
    constantPitchPropagator,
    constantThrustPropagator,
    FlightPathAnglePitchTarget,
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
    PropagatorOptions,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { GeometricPathPoint } from '@fmgc/flightmanagement/vnav/segments/GeometricPathSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { MathUtils } from '@shared/MathUtils';

export class PureConstantFlightPathAngleSegment extends ProfileSegment {
    private integrator = new Integrator();

    private endConditions: IntegrationEndConditions;

    private idlePropagator: IntegrationPropagator;

    private fpaPropagator: IntegrationPropagator;

    /**
     *
     * @param context
     * @param flightPathAngle
     * @param toDistance
     * @param maxAltiude
     */
    constructor(
        context: SegmentContext,
        private flightPathAngle: Degrees,
        private toDistance: NauticalMiles,
        options: PropagatorOptions,
        private maxAltiude: Feet = context.observer.get().cruiseAltitude,
    ) {
        super();

        this.endConditions = {
            distanceFromStart: { min: toDistance },
            altitude: { max: this.maxAltiude },
        };

        this.fpaPropagator = constantPitchPropagator(new FlightPathAnglePitchTarget(flightPathAngle), context, options);
        this.idlePropagator = constantThrustPropagator(new IdleThrustSetting(context.atmosphericConditions), context, options);
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        // - Run idle prediction.
        //  - If we achive the desired FPA at idle, that's good. We can then run the prediction at non idle with the desired FPA
        //  - If we don't achieve it at idle, try idle with speed brakes
        //      - If we succeed with speed brakes, run the prediction for desired FPA with speed brakes
        //      - If we don't succeed at idle with speed brakes, insert PATH TOO STEEP

        const idlePath = this.integrator.integrate(
            state,
            this.endConditions,
            this.idlePropagator,
        );

        // This means that an endcondition is already met. In this case, the entire FPA segment can be skipped.
        if (idlePath.length <= 1) {
            return;
        }

        const achievedGradient = this.calculateGradient(idlePath.first, idlePath.last);
        const achievedFpa = MathUtils.RADIANS_TO_DEGREES * Math.atan(achievedGradient / 6076.12);

        // `achievedFpa` and `this.flightPathAngle` are negative
        if (achievedFpa - this.flightPathAngle > 0.1) {
            // We didn't make it -> Try with speed brakes
            console.log(`[FMS/VNAV] Desired path angle of ${this.flightPathAngle}° could not be achieved, actual path angle: ${achievedFpa}°. Trying with speedbrakes`);
            state.config.speedbrakesExtended = true;

            const idlePathWithSpeedBrakes = this.integrator.integrate(
                state,
                this.endConditions,
                this.idlePropagator,
            );

            const achievedGradientWithSpeedBrakes = this.calculateGradient(idlePathWithSpeedBrakes.first, idlePathWithSpeedBrakes.last);
            const achievedFpaWithSpeedBrakes = MathUtils.RADIANS_TO_DEGREES * Math.atan(achievedGradientWithSpeedBrakes / 6076.12);

            console.log(`[FMS/VNAV] Achieved path angle of ${achievedFpaWithSpeedBrakes} with speedbrakes`);

            if (achievedFpaWithSpeedBrakes - this.flightPathAngle > 0.1) {
                console.log(`[FMS/VNAV] TOO STEEP PATH: Desired FPA of ${this.flightPathAngle}° but only achieved ${achievedFpaWithSpeedBrakes}°.`);
                // Insert TOO PATH STEEP
            }
        }

        const descentPath = this.integrator.integrate(
            state,
            this.endConditions,
            this.fpaPropagator,
        );

        // Retract speed brakes again
        state.config.speedbrakesExtended = false;

        if (descentPath.length >= 1) {
            builder.push(descentPath.last);
        }
    }

    private calculateGradient(start: GeometricPathPoint, end: GeometricPathPoint) {
        return Math.abs(start.distanceFromStart - end.distanceFromStart) < 1e-12
            ? 0
            : (start.altitude - end.altitude) / (start.distanceFromStart - end.distanceFromStart);
    }

    get repr(): string {
        return `PureConstantFlightPathAngleSegment - Descend from ${this.toDistance.toFixed(2)} NM`;
    }
}
