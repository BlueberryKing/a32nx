import { CpuTimer, measurePerformance } from '@fmgc/flightmanagement/vnav/common/profiling';
import { AircraftState, SegmentContext, TemporaryStateSequence } from '@fmgc/flightmanagement/vnav/segments';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { AccelFactorMode, Common } from '@fmgc/guidance/vnav/common';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { FlightModel } from '@fmgc/guidance/vnav/FlightModel';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { MathUtils } from '@shared/MathUtils';
import { WindProfileType } from '../../../guidance/vnav/wind/WindProfile';

export interface PropagatorOptions {
    stepSize: NauticalMiles;
    useMachVsCas: boolean;
    windProfileType: WindProfileType;
}

export function constantThrustPropagator(thrustSetting: ThrustSetting, context: SegmentContext, options: PropagatorOptions) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);

        const headwind = context.windRepository.getWindProfile(options.windProfileType).getHeadwindComponent(state.distanceFromStart, state.altitude);
        const groundSpeed = state.trueAirspeed - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            options.useMachVsCas ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS,
        );

        const [thrust, fuelFlow] = thrustSetting.getThrustAndFuelFlow(state);
        const pathAngle: Radians = FlightModel.getConstantThrustPathAngle(thrust, state.weight, drag, accelerationFactor);
        const verticalSpeed: FeetPerMinute = 101.268 * state.trueAirspeed * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * options.stepSize / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);
        const newTheta = Common.getTheta(newAltitude, context.getIsaDeviation());

        const newCas = options.useMachVsCas ? Common.machToCas(state.mach, newDelta) : state.speed;
        const newMach = options.useMachVsCas ? state.mach : Common.CAStoMach(state.speed, newDelta);

        return {
            distanceFromStart: state.distanceFromStart + options.stepSize,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: newCas,
            trueAirspeed: options.useMachVsCas ? Common.machToTAS(newMach, newTheta) : Common.CAStoTAS(newCas, newTheta, newDelta),
            mach: newMach,
            config: state.config,
        };
    };
}

export function constantPitchPropagator(pitch: PitchTarget, context: SegmentContext, options: PropagatorOptions) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const theta = Common.getTheta(state.altitude, context.getIsaDeviation(), state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);
        const delta2 = Common.getDelta2(delta, state.mach);
        const theta2 = Common.getTheta2(theta, state.mach);

        const headwind = context.windRepository.getWindProfile(options.windProfileType).getHeadwindComponent(state.distanceFromStart, state.altitude);
        const groundSpeed = state.trueAirspeed - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            options.useMachVsCas ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS,
        );

        const pathAngle: Radians = pitch.getPathAngle(state);
        const thrust = FlightModel.getThrustFromConstantPathAngle(pathAngle * MathUtils.RADIANS_TO_DEGREES, state.weight, drag, accelerationFactor);
        const correctedThrust = (thrust / delta2) / 2;
        const n1 = EngineModel.reverseTableInterpolation(EngineModel.table1506, state.mach, (correctedThrust / EngineModel.maxThrust));
        const correctedN1 = EngineModel.getCorrectedN1(n1, theta2);
        const fuelFlow = EngineModel.getCorrectedFuelFlow(correctedN1, state.mach, state.altitude) * 2;

        const verticalSpeed: FeetPerMinute = 101.268 * state.trueAirspeed * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * options.stepSize / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);
        const newTheta = Common.getTheta(newAltitude, context.getIsaDeviation());

        const newCas = options.useMachVsCas ? Common.machToCas(state.mach, newDelta) : state.speed;
        const newMach = options.useMachVsCas ? state.mach : Common.CAStoMach(state.speed, newDelta);

        return {
            distanceFromStart: state.distanceFromStart + options.stepSize,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: newCas,
            trueAirspeed: options.useMachVsCas ? Common.machToTAS(newMach, newTheta) : Common.CAStoTAS(newCas, newTheta, newDelta),
            mach: newMach,
            config: state.config,
        };
    };
}

export function accelerationPropagator(thrustSetting: ThrustSetting, context: SegmentContext, options: PropagatorOptions) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);

        const headwind = context.windRepository.getWindProfile(options.windProfileType).getHeadwindComponent(state.distanceFromStart, state.altitude);
        const groundSpeed = state.trueAirspeed - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            options.useMachVsCas ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS,
        );

        const [thrust, fuelFlow] = thrustSetting.getThrustAndFuelFlow(state);

        const availableGradient: Radians = FlightModel.getAvailableGradient(thrust, drag, state.weight);
        const pathAngle: Radians = FlightModel.getSpeedChangePathAngle(thrust, state.weight, drag);
        const acceleration: KnotsPerSecond = FlightModel.accelerationForGradient(availableGradient, pathAngle, accelerationFactor) * FlightModel.gravityConstKNS;
        const verticalSpeed: FeetPerMinute = 101.268 * state.trueAirspeed * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * options.stepSize / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newTas = state.trueAirspeed + acceleration * stepTime;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);
        const newTheta = Common.getTheta(newAltitude, context.getIsaDeviation());

        return {
            distanceFromStart: state.distanceFromStart + options.stepSize,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: Common.TAStoCAS(newTas, newTheta, newDelta),
            trueAirspeed: newTas,
            mach: Common.TAStoMach(newTas, newTheta),
            config: state.config,
        };
    };
}

export function speedChangePropagator(context: SegmentContext, thrustSetting: ThrustSetting, pitchTarget: PitchTarget, desireAccelerationVsDeceleration: boolean, options: PropagatorOptions) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);

        const headwind = context.windRepository.getWindProfile(options.windProfileType).getHeadwindComponent(state.distanceFromStart, state.altitude);
        const groundSpeed = state.trueAirspeed - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            options.useMachVsCas ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS,
        );

        const [thrust, fuelFlow] = thrustSetting.getThrustAndFuelFlow(state);

        const availableGradient: Radians = FlightModel.getAvailableGradient(thrust, drag, state.weight);

        // I don't know if this exists in the real aircraft. I suspect there is, but these are just empirical values.
        const minimumAccelDecel: KnotsPerSecond = desireAccelerationVsDeceleration ? 0.5 : -0.3;

        // The reason this is here is that we might be demanding a path angle which doesn't actually end up accelerating/decelerating the plane.
        const pathAngleForMinimumAccelDecel = FlightModel.fpaForGradient(availableGradient, minimumAccelDecel / FlightModel.gravityConstKNS, accelerationFactor);

        const targetPathAngle: Radians = pitchTarget.getPathAngle(state);
        const pathAngle = desireAccelerationVsDeceleration
            ? Math.max(0, Math.min(targetPathAngle, pathAngleForMinimumAccelDecel))
            : Math.min(0, Math.max(targetPathAngle, pathAngleForMinimumAccelDecel));

        const acceleration: KnotsPerSecond = FlightModel.accelerationForGradient(availableGradient, pathAngle, accelerationFactor) * FlightModel.gravityConstKNS;
        const verticalSpeed: FeetPerMinute = 101.268 * state.trueAirspeed * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * options.stepSize / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newTas = state.trueAirspeed + acceleration * stepTime;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);
        const newTheta = Common.getTheta(newAltitude, context.getIsaDeviation());

        return {
            distanceFromStart: state.distanceFromStart + options.stepSize,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: Common.TAStoCAS(newTas, newTheta, newDelta),
            trueAirspeed: newTas,
            mach: Common.TAStoMach(newTas, newTheta),
            config: state.config,
        };
    };
}

export interface PitchTarget {
    getPathAngle(state: AircraftState): Radians;
}

export class VerticalSpeedPitchTarget implements PitchTarget {
    constructor(private verticalSpeed: FeetPerMinute) { }

    getPathAngle({ trueAirspeed }: AircraftState): Radians {
        return Math.atan2(this.verticalSpeed, trueAirspeed * 101.269); // radians
    }
}

export class FlightPathAnglePitchTarget implements PitchTarget {
    constructor(private flightPathAngle: Degrees) { }

    getPathAngle(_: AircraftState): Radians {
        return this.flightPathAngle * MathUtils.DEGREES_TO_RADIANS;
    }
}

export interface ThrustSetting {
    getThrustAndFuelFlow(state: AircraftState): [number, number]
}

export class ClimbThrustSetting implements ThrustSetting {
    constructor(private atmosphericConditions: AtmosphericConditions) {

    }

    getThrustAndFuelFlow({ altitude, mach }: AircraftState): [number, number] {
        const estimatedTat = this.atmosphericConditions.totalAirTemperatureFromMach(altitude, mach);
        const n1 = EngineModel.tableInterpolation(EngineModel.maxClimbThrustTableLeap, estimatedTat, altitude);

        const theta = Common.getTheta(altitude, this.atmosphericConditions.isaDeviation);
        const delta = Common.getDelta(altitude, false);
        const theta2 = Common.getTheta2(theta, mach);
        const delta2 = Common.getDelta2(delta, mach);
        const correctedN1 = EngineModel.getCorrectedN1(n1, theta2);
        const correctedThrust = EngineModel.tableInterpolation(EngineModel.table1506, correctedN1, mach) * 2 * EngineModel.maxThrust;

        return [
            EngineModel.getUncorrectedThrust(correctedThrust, delta2), // in lbf
            EngineModel.getCorrectedFuelFlow(correctedN1, mach, altitude) * 2,
        ];
    }
}

export class TakeoffThrustSetting implements ThrustSetting {
    constructor(private atmosphericConditions: AtmosphericConditions) {

    }

    getThrustAndFuelFlow({ altitude, mach }: AircraftState): [number, number] {
        const n1 = SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_THRUST_LIMIT_TOGA', 'Percent');

        const theta = Common.getTheta(altitude, this.atmosphericConditions.isaDeviation);
        const delta = Common.getDelta(altitude, false);
        const theta2 = Common.getTheta2(theta, mach);
        const delta2 = Common.getDelta2(delta, mach);
        const correctedN1 = EngineModel.getCorrectedN1(n1, theta2);
        const correctedThrust = EngineModel.tableInterpolation(EngineModel.table1506, correctedN1, mach) * 2 * EngineModel.maxThrust;

        return [
            EngineModel.getUncorrectedThrust(correctedThrust, delta2), // in lbf
            EngineModel.getCorrectedFuelFlow(correctedN1, mach, altitude) * 2,
        ];
    }
}

export class IdleThrustSetting implements ThrustSetting {
    constructor(private atmosphericConditions: AtmosphericConditions) {

    }

    getThrustAndFuelFlow({ altitude, mach }: AircraftState): [number, number] {
        const n1 = EngineModel.getIdleN1(altitude, mach) + VnavConfig.IDLE_N1_MARGIN;

        const theta = Common.getTheta(altitude, this.atmosphericConditions.isaDeviation);
        const delta = Common.getDelta(altitude, false);
        const theta2 = Common.getTheta2(theta, mach);
        const delta2 = Common.getDelta2(delta, mach);
        const correctedN1 = EngineModel.getCorrectedN1(n1, theta2);
        const correctedThrust = EngineModel.tableInterpolation(EngineModel.table1506, correctedN1, mach) * 2 * EngineModel.maxThrust;

        return [
            EngineModel.getUncorrectedThrust(correctedThrust, delta2), // in lbf
            EngineModel.getCorrectedFuelFlow(correctedN1, mach, altitude) * 2,
        ];
    }
}

export type IntegrationPropagator = (state: AircraftState) => AircraftState;

type MinMax = { min?: number, max?: number };
export type IntegrationEndConditions = Partial<Record<keyof AircraftState, MinMax>>;

export class Integrator {
    integrate(startingState: AircraftState, endConditions: IntegrationEndConditions, propagator: IntegrationPropagator): TemporaryStateSequence {
        return measurePerformance(() => this.integrateInternal(startingState, endConditions, propagator), (time, result) => {
            CpuTimer.integrationTime += time;
            CpuTimer.integrationSteps += result.length - 1;
        });
    }

    private integrateInternal(startingState: AircraftState, endConditions: IntegrationEndConditions, propagator: IntegrationPropagator): TemporaryStateSequence {
        const states = new TemporaryStateSequence(startingState);

        if (!this.checkEndConditions(startingState, endConditions).next().done) {
            return states;
        }

        let isEndConditionMet = false;
        for (let i = 0; !isEndConditionMet && i < 1000; i++) {
            const newState = propagator(states.last);

            // For all endconditions that are met, we scale to accomodate them and then push the state to the sequence and return.
            for (const metEndCondition of this.checkEndConditions(newState, endConditions)) {
                isEndConditionMet = true;
                const [key, value] = metEndCondition;

                const scalingConstant = (value - states.last[key]) / (newState[key] - states.last[key]);
                this.scaleState(states.last, newState, scalingConstant);
            }

            states.push(newState);
        }

        return states;
    }

    private* checkEndConditions(state: AircraftState, endConditions: IntegrationEndConditions): Generator<[string, number]> {
        for (const [key, limits] of Object.entries(endConditions)) {
            if (isNumber(limits.min) && state[key] - limits.min <= 1e-9) {
                yield [key, limits.min];
            } if (isNumber(limits.max) && state[key] - limits.max >= -1e-9) {
                yield [key, limits.max];
            }
        }
    }

    private scaleState(secondLastState: AircraftState, state: AircraftState, scalingConstant: number) {
        state.altitude = (1 - scalingConstant) * secondLastState.altitude + scalingConstant * state.altitude;
        state.distanceFromStart = (1 - scalingConstant) * secondLastState.distanceFromStart + scalingConstant * state.distanceFromStart;
        state.mach = (1 - scalingConstant) * secondLastState.mach + scalingConstant * state.mach;
        state.speed = (1 - scalingConstant) * secondLastState.speed + scalingConstant * state.speed;
        state.time = (1 - scalingConstant) * secondLastState.time + scalingConstant * state.time;
        state.trueAirspeed = (1 - scalingConstant) * secondLastState.trueAirspeed + scalingConstant * state.trueAirspeed;
        state.weight = (1 - scalingConstant) * secondLastState.weight + scalingConstant * state.weight;
    }
}

function isNumber(value: number): boolean {
    return !Number.isNaN(value) && value !== undefined && value !== null;
}
