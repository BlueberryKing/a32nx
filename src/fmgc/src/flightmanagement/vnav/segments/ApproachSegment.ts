import { constantPitchPropagator, FlightPathAnglePitchTarget, IdleThrustSetting, IntegrationEndCondition, IntegrationPropagator, Integrator, speedChangePropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ConfigurationChangeSegment } from '@fmgc/flightmanagement/vnav/segments/ConfigurationChangeSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureConstantFlightPathAngleSegment } from '@fmgc/flightmanagement/vnav/segments/PureConstantFlightPathAngleSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { DescentAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { MathUtils } from '@shared/MathUtils';

/**
 * This represents a path from the Missed Approach Point to the Decel point, slowing the aircraft from descent speed to Vapp.
 */
export class ApproachSegment extends ProfileSegment {
    constructor(context: NodeContext, constraints: ConstraintReader) {
        super();

        const { cleanSpeed, slatRetractionSpeed, flapRetractionSpeed, approachSpeed, isFlaps3Landing } = context.observer.get();

        this.children = [
            new FinalApproachSegment(context),
            new ApproachFlapSegment(context, constraints, flapRetractionSpeed), // In flaps 3
            new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_2, gearExtended: true }),
            new ApproachFlapSegment(context, constraints, slatRetractionSpeed), // In flaps 2
            new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_1 }),
            new ApproachFlapSegment(context, constraints, cleanSpeed), // In flaps 1

        ];

        if (!isFlaps3Landing) {
            this.children.unshift(
                new ApproachFlapSegment(context, constraints, (flapRetractionSpeed + approachSpeed) / 2),
            );

            this.children.unshift(
                new ConfigurationChangeSegment(context, { flapConfig: FlapConf.CONF_3 }),
            );
        }
    }

    get repr(): string {
        return 'ApproachSegment';
    }
}

export class ApproachFlapSegment extends ProfileSegment {
    constructor(private context: NodeContext, private constraints: ConstraintReader, private nextFlapSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        // Why this? This is about -500 fpm at 200 kts, which should achieve decent deceleration
        // TODO: Use better method for this.
        const preferredFlightPathAngle: Degrees = -1.5;
        let maxSpeed = this.nextFlapSpeed;

        this.children = [
            new PureApproachDecelerationSegment(
                this.context,
                preferredFlightPathAngle,
                this.nextFlapSpeed,
                -Infinity,
            ),
        ];

        // Speed constraints appearing before any altitude constraints, which inhibit a direct deceleration to the flap extension speed
        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            // If the constraint is "behind" the aircraft in the reverse profile, we ignore it
            // Also, if the constraint is between two altitude constraints, it will be treated further down.
            if (state.distanceFromStart <= speedConstraint.distanceFromStart
                || this.constraints.descentAltitudeConstraints.length > 0 && speedConstraint.distanceFromStart >= this.constraints.descentAltitudeConstraints[0].distanceFromStart
            ) {
                break;
            // If the constraint does not actually stop us from "accelerating" to the flap extension speed, ignore it
            } else if (speedConstraint.maxSpeed > this.nextFlapSpeed) {
                continue;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureApproachDecelerationSegment(this.context, preferredFlightPathAngle, maxSpeed, speedConstraint.distanceFromStart),
            );

            this.children.unshift(
                new PureConstantFlightPathAngleSegment(this.context, preferredFlightPathAngle, speedConstraint.distanceFromStart),
            );
        }

        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.distanceFromStart > state.distanceFromStart) {
                continue;
            }

            this.children.unshift(
                new ApproachAltitudeConstraintSegment(this.context, this.constraints, constraint, preferredFlightPathAngle, maxSpeed),
            );
        }
    }

    get repr(): string {
        return 'ApproachFlapSegment';
    }
}

export class PureApproachConstantSpeedSegment extends ProfileSegment {
    private integrator = new Integrator();

    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toDistance: NauticalMiles, private toSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const endConditions = [
            ({ distanceFromStart }) => distanceFromStart <= this.toDistance,
            ({ speed }) => speed >= this.toSpeed,
        ];

        const pitchTarget = new FlightPathAnglePitchTarget(this.flightPathAngle);

        const descentPath = this.integrator.integrate(
            state,
            endConditions,
            constantPitchPropagator(pitchTarget, this.context, -0.1),
        );

        if (descentPath.length > 1) {
            builder.push(descentPath.last);
        }
    }

    get repr(): string {
        return 'PureApproachConstantSpeedSegment';
    }
}

export class PureApproachDecelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndCondition[];

    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toSpeed: Knots, private toDistance) {
        super();

        this.endConditions = [
            ({ distanceFromStart }) => distanceFromStart <= toDistance,
            ({ speed }) => speed >= toSpeed,
        ];
    }

    get repr(): string {
        return 'PureApproachDecelerationSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const propagator = speedChangePropagator(
            new IdleThrustSetting(this.context.atmosphericConditions),
            new FlightPathAnglePitchTarget(this.flightPathAngle),
            this.context,
            -0.1,
        );

        const step = this.integrator.integrate(
            state,
            this.endConditions,
            propagator,
        );

        if (step.length > 1) {
            builder.push(step.last);
        }
    }
}

export class ApproachAltitudeConstraintSegment extends ProfileSegment {
    constructor(
        private context: NodeContext, private constraints: ConstraintReader, private constraint: DescentAltitudeConstraint, private preferredFlightPathAngle: Degrees, private maxSpeed: Knots,
    ) {
        super();
    }

    get repr(): string {
        return 'ApproachAltitudeConstraintSegment';
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        // TODO: Try get FPA from previous segment to minimize pitch changes.
        const [minAngle, maxAngle] = this.getFlightPathAngleRange(state, this.constraint);

        const flightPathAngle = Math.max(minAngle, Math.min(maxAngle, this.preferredFlightPathAngle));
        let maxSpeed = this.maxSpeed;

        this.children = [
            new PureApproachDecelerationSegment(this.context, flightPathAngle, maxSpeed, this.constraint.distanceFromStart),
        ];

        for (const speedConstraint of this.constraints.descentSpeedConstraints) {
            if (speedConstraint.distanceFromStart < this.constraint.distanceFromStart || speedConstraint.maxSpeed > this.maxSpeed) {
                continue;
            } else if (speedConstraint.distanceFromStart > state.distanceFromStart) {
                break;
            }

            maxSpeed = Math.min(maxSpeed, speedConstraint.maxSpeed);

            this.children.unshift(
                new PureConstantFlightPathAngleSegment(this.context, flightPathAngle, speedConstraint.distanceFromStart),
            );

            this.children.unshift(
                new PureApproachDecelerationSegment(this.context, flightPathAngle, maxSpeed, speedConstraint.distanceFromStart),
            );
        }
    }

    private getFlightPathAngleRange(state: AircraftState, constraint: DescentAltitudeConstraint): [Degrees, Degrees] {
        switch (constraint.constraint.type) {
        case AltitudeConstraintType.at:
            return [
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
            ];
        case AltitudeConstraintType.atOrAbove:
            return [
                -20,
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
            ];
        case AltitudeConstraintType.atOrBelow:
            return [
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
                0,
            ];
        case AltitudeConstraintType.range:
            return [
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude1, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
                MathUtils.RADIANS_TO_DEGREES * Math.atan2(state.altitude - constraint.constraint.altitude2, 6076.12 * (state.distanceFromStart - constraint.distanceFromStart)),
            ];
        default:
            console.error('Invalid altitude constraint type');
            return null;
        }
    }
}

/**
 * Represents segment from 1000 ft above the threshold to the Missed Approach Point
 */
export class StabilizedFinalApproachSegment extends ProfileSegment {
    get repr(): string {
        return 'StabilizedFinalApproachSegment';
    }
}

/**
 * Represents Segment from Final Descent Point to 1000 ft above the threshold
 */
export class FinalApproachSegment extends ProfileSegment {
    private integrator: Integrator

    private endConditions: IntegrationEndCondition[]

    private propagator: IntegrationPropagator;

    constructor(context: NodeContext) {
        super();

        const { destinationAirfieldElevation } = context.observer.get();

        this.integrator = new Integrator();

        this.endConditions = [
            ({ altitude }) => altitude > destinationAirfieldElevation + 1000,
        ];

        this.propagator = constantPitchPropagator(
            new FlightPathAnglePitchTarget(-3),
            context,
            -0.1,
        );
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const segment = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        );

        if (segment.length > 1) {
            builder.push(segment.last);
        }
    }

    get repr(): string {
        return 'FinalApproachSegment';
    }
}
