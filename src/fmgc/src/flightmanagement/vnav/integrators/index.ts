import { AircraftState, NodeContext } from '@fmgc/flightmanagement/vnav/segments';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { AccelFactorMode, Common } from '@fmgc/guidance/vnav/common';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { FlightModel } from '@fmgc/guidance/vnav/FlightModel';

export function constantThrustPropagator(thrustSetting: ThrustSetting, context: NodeContext) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const theta = Common.getTheta(state.altitude, context.getIsaDeviation(), state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);

        const headwind = context.windProfile.getHeadwindComponent(state.distanceFromStart, state.altitude);
        const tas = Common.CAStoTAS(state.speed, theta, delta);
        const groundSpeed = tas - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            AccelFactorMode.CONSTANT_CAS,
        );

        const [thrust, fuelFlow] = thrustSetting.getThrustAndFuelFlow(state);
        const distance: NauticalMiles = 0.1;
        const pathAngle: Radians = FlightModel.getConstantThrustPathAngle(thrust, state.weight, drag, accelerationFactor);
        const verticalSpeed: FeetPerMinute = 101.268 * tas * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * distance / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);

        return {
            distanceFromStart: state.distanceFromStart + distance,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: state.speed,
            trueAirspeed: state.trueAirspeed,
            mach: Common.CAStoMach(state.speed, newDelta),
            config: state.config,
        };
    };
}

export function constantPitchPropagator(pitch: PitchTarget, context: NodeContext) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const theta = Common.getTheta(state.altitude, context.getIsaDeviation(), state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);
        const delta2 = Common.getDelta2(delta, state.mach);
        const theta2 = Common.getTheta2(theta, state.mach);

        const headwind = context.windProfile.getHeadwindComponent(state.distanceFromStart, state.altitude);
        const tas = Common.CAStoTAS(state.speed, theta, delta);
        const groundSpeed = tas - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            AccelFactorMode.CONSTANT_CAS,
        );

        const pathAngle = pitch.getPathAngle(state);
        const thrust = FlightModel.getThrustFromConstantPathAngle(pathAngle, state.weight, drag, accelerationFactor);
        const correctedThrust = (thrust / delta2) / 2;

        const n1 = EngineModel.reverseTableInterpolation(EngineModel.table1506, state.mach, (correctedThrust / EngineModel.maxThrust));
        const correctedN1 = EngineModel.getCorrectedN1(n1, theta2);
        const fuelFlow = EngineModel.getCorrectedFuelFlow(correctedN1, state.mach, state.altitude) * 2;

        const distance: NauticalMiles = 0.1;
        const verticalSpeed: FeetPerMinute = 101.268 * tas * Math.sin(pathAngle);
        const stepTime: Seconds = distance / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);

        return {
            distanceFromStart: state.distanceFromStart + distance,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: state.speed,
            trueAirspeed: state.trueAirspeed,
            mach: Common.CAStoMach(state.speed, newDelta),
            config: state.config,
        };
    };
}

export function accelerationPropagator(thrustSetting: ThrustSetting, context: NodeContext) {
    const { tropoPause } = context.observer.get();

    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const theta = Common.getTheta(state.altitude, context.getIsaDeviation(), state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);

        const headwind = context.windProfile.getHeadwindComponent(state.distanceFromStart, state.altitude);
        const tas = Common.CAStoTAS(state.speed, theta, delta);
        const groundSpeed = tas - headwind.value;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            context.getIsaDeviation(),
            state.altitude > tropoPause,
            AccelFactorMode.CONSTANT_CAS,
        );

        const [thrust, fuelFlow] = thrustSetting.getThrustAndFuelFlow(state);
        const distance: NauticalMiles = 0.1;

        const availableGradient: Radians = FlightModel.getAvailableGradient(thrust, drag, state.weight);
        const pathAngle: Radians = FlightModel.getSpeedChangePathAngle(thrust, state.weight, drag);
        const acceleration: KnotsPerSecond = FlightModel.accelerationForGradient(availableGradient, pathAngle, accelerationFactor) * FlightModel.gravityConstKNS;
        const verticalSpeed: FeetPerMinute = 101.268 * tas * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * distance / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        const newAltitude = state.altitude + stepTime / 60 * verticalSpeed;
        const newCas = state.speed + acceleration * stepTime;
        const newDelta = Common.getDelta(newAltitude, newAltitude > tropoPause);

        return {
            distanceFromStart: state.distanceFromStart + distance,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: state.speed + acceleration * stepTime,
            trueAirspeed: state.trueAirspeed,
            mach: Common.CAStoMach(newCas, newDelta),
            config: state.config,
        };
    };
}

interface PitchTarget {
    getPathAngle(state: AircraftState): Radians;
}

class VerticalSpeedPitchTarget implements PitchTarget {
    constructor(private verticalSpeed: FeetPerMinute) { }

    getPathAngle({ trueAirspeed }: AircraftState): Radians {
        return Math.atan2(this.verticalSpeed, trueAirspeed * 101.269); // radians
    }
}

export class FlightPathAnglePitchTarget implements PitchTarget {
    constructor(private flightPathAngle: Degrees) { }

    getPathAngle(_: AircraftState): Radians {
        return this.flightPathAngle;
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

export type IntegrationEndCondition = (state: AircraftState) => boolean;
type IntegrationPropagator = (state: AircraftState) => AircraftState;

export class Integrator {
    integrate(startingState: AircraftState, endConditions: IntegrationEndCondition[], propagator: IntegrationPropagator) {
        if (endConditions.some((condition) => condition(startingState))) {
            return startingState;
        }

        let finalState = startingState;
        let i = 0;

        while (i++ < 1000) {
            finalState = propagator(finalState);

            if (endConditions.some((condition) => condition(finalState))) {
                return finalState;
            }
        }

        return finalState;
    }
}
