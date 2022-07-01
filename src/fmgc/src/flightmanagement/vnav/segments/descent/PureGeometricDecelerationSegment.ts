import {
    FlightPathAnglePitchTarget,
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
    PropagatorOptions,
    speedChangePropagator,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { GeometricPathPoint } from '@fmgc/flightmanagement/vnav/segments/descent/GeometricPathSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { MathUtils } from '@shared/MathUtils';

export class PureGeometricDecelerationSegment extends ProfileSegment {
    private integrator = new Integrator();

    private propagator: IntegrationPropagator;

    private managedDescentSpeedMach: Mach = 0.82;

    constructor(context: SegmentContext, private flightPathAngle: Degrees, private toSpeed: Knots, private toDistance: NauticalMiles, private maxAltitude: Knots, options: PropagatorOptions) {
        super();

        this.propagator = speedChangePropagator(
            context,
            new IdleThrustSetting(context.atmosphericConditions),
            new FlightPathAnglePitchTarget(flightPathAngle),
            false,
            options,
        );

        const { managedDescentSpeedMach } = context.observer.get();
        this.managedDescentSpeedMach = managedDescentSpeedMach;
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        // The AMM says decel distance is max 20 NM and goes max 6000 higher.
        const endConditions: IntegrationEndConditions = {
            distanceFromStart: { min: Math.max(this.toDistance, state.distanceFromStart - 20) },
            altitude: { max: Math.min(this.maxAltitude, state.altitude + 6000) },
            speed: { max: this.toSpeed },
            mach: { max: this.managedDescentSpeedMach },
        };

        const decelerationPath = this.integrator.integrate(
            state,
            endConditions,
            this.propagator,
        );

        if (decelerationPath.length <= 1) {
            return;
        }

        const achievedGradient = this.calculateGradient(decelerationPath.first, decelerationPath.last);
        const achievedFpa = MathUtils.RADIANS_TO_DEGREES * Math.atan(achievedGradient / 6076.12);

        // `achievedFpa` and `this.flightPathAngle` are negative
        if (achievedFpa - this.flightPathAngle > 0.1) {
            // We didn't make it -> Try with speed brakes
            state.config.speedbrakesExtended = true;

            const decelerationPathWithSpeedBrakes = this.integrator.integrate(
                state,
                endConditions,
                this.propagator,
            );

            const achievedGradientWithSpeedBrakes = this.calculateGradient(decelerationPathWithSpeedBrakes.first, decelerationPathWithSpeedBrakes.last);
            const achievedFpaWithSpeedBrakes = MathUtils.RADIANS_TO_DEGREES * Math.atan(achievedGradientWithSpeedBrakes / 6076.12);

            if (achievedFpaWithSpeedBrakes - this.flightPathAngle > 0.1) {
                console.log(`[FMS/VNAV] TOO STEEP PATH: Desired FPA of ${this.flightPathAngle}° but only achieved ${achievedFpaWithSpeedBrakes}°.`);
                // Insert TOO PATH STEEP
            }

            if (decelerationPathWithSpeedBrakes.length > 1) {
                builder.push({ ...decelerationPathWithSpeedBrakes.last, reason: 'FPA decel with speedbrakes' });
                state.config.speedbrakesExtended = false;
            }
        } else {
            builder.push({ ...decelerationPath.last, reason: 'FPA decel' });
        }
    }

    private calculateGradient(start: GeometricPathPoint, end: GeometricPathPoint) {
        return Math.abs(start.distanceFromStart - end.distanceFromStart) < 1e-12
            ? 0
            : (start.altitude - end.altitude) / (start.distanceFromStart - end.distanceFromStart);
    }

    get repr(): string {
        return `PureGeometricDecelerationSegment - Decelerate to ${this.toSpeed.toFixed(0)} kts`;
    }
}
