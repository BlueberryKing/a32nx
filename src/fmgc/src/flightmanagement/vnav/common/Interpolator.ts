import { AircraftState, AircraftStateWithPhase } from '@fmgc/flightmanagement/vnav/segments';
import { Common } from '@fmgc/guidance/vnav/common';

export class Interpolator {
    private static interpolateStates<T extends number, U extends number>(
        states: AircraftState[], indexValue: T, keySelector: (checkpoint: AircraftState) => T, valueSelector: (checkpoint: AircraftState) => U,
    ) {
        if (indexValue <= keySelector(states[0])) {
            return valueSelector(states[0]);
        }

        for (let i = 0; i < states.length - 1; i++) {
            if (indexValue > keySelector(states[i]) && indexValue <= keySelector(states[i + 1])) {
                return Common.interpolate(
                    indexValue,
                    keySelector(states[i]),
                    keySelector(states[i + 1]),
                    valueSelector(states[i]),
                    valueSelector(states[i + 1]),
                );
            }
        }

        return valueSelector(states[states.length - 1]);
    }

    private static interpolateStatesBackwards<T extends number, U extends number>(
        states: AircraftState[], indexValue: T, keySelector: (checkpoint: AircraftState) => T, valueSelector: (checkpoint: AircraftState) => U,
    ) {
        if (indexValue < keySelector(states[states.length - 1])) {
            return valueSelector(states[states.length - 1]);
        }

        for (let i = states.length - 2; i >= 0; i--) {
            if (indexValue <= keySelector(states[i]) && indexValue > keySelector(states[i + 1])) {
                return Common.interpolate(
                    indexValue,
                    keySelector(states[i]),
                    keySelector(states[i + 1]),
                    valueSelector(states[i]),
                    valueSelector(states[i + 1]),
                );
            }
        }

        return valueSelector(states[0]);
    }

    static interpolateDistanceAtAltitude(states: AircraftStateWithPhase[], altitude: Feet, reverse: boolean = false): NauticalMiles {
        return reverse
            ? this.interpolateStatesBackwards(states, altitude, (state) => state.altitude, (state) => state.distanceFromStart)
            : this.interpolateStates(states, altitude, (state) => state.altitude, (state) => state.distanceFromStart);
    }

    static interpolateEverythingFromStart(states: AircraftStateWithPhase[], distanceFromStart: NauticalMiles): AircraftStateWithPhase {
        if (distanceFromStart <= states[0].distanceFromStart) {
            return {
                distanceFromStart,
                altitude: states[0].altitude,
                time: states[0].time,
                speeds: {
                    calibratedAirspeed: states[0].speeds.calibratedAirspeed,
                    mach: states[0].speeds.mach,
                    trueAirspeed: states[0].speeds.trueAirspeed,
                    groundSpeed: states[0].speeds.groundSpeed,
                    speedTarget: states[0].speeds.speedTarget,
                    speedTargetType: states[0].speeds.speedTargetType,
                },
                weight: states[0].weight,
                config: states[0].config,
                phase: states[0].phase,
            };
        }

        for (let i = 0; i < states.length - 1; i++) {
            if (distanceFromStart > states[i].distanceFromStart && distanceFromStart <= states[i + 1].distanceFromStart) {
                return {
                    distanceFromStart,
                    altitude: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].altitude,
                        states[i + 1].altitude,
                    ),
                    time: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].time,
                        states[i + 1].time,
                    ),
                    speeds: {
                        calibratedAirspeed: Common.interpolate(
                            distanceFromStart,
                            states[i].distanceFromStart,
                            states[i + 1].distanceFromStart,
                            states[i].speeds.calibratedAirspeed,
                            states[i + 1].speeds.calibratedAirspeed,
                        ),
                        mach: Common.interpolate(
                            distanceFromStart,
                            states[i].distanceFromStart,
                            states[i + 1].distanceFromStart,
                            states[i].speeds.mach,
                            states[i + 1].speeds.mach,
                        ),
                        trueAirspeed: Common.interpolate(
                            distanceFromStart,
                            states[i].distanceFromStart,
                            states[i + 1].distanceFromStart,
                            states[i].speeds.trueAirspeed,
                            states[i + 1].speeds.trueAirspeed,
                        ),
                        groundSpeed: Common.interpolate(
                            distanceFromStart,
                            states[i].distanceFromStart,
                            states[i + 1].distanceFromStart,
                            states[i].speeds.groundSpeed,
                            states[i + 1].speeds.groundSpeed,
                        ),
                        speedTarget: states[i + 1].speeds.speedTarget,
                        speedTargetType: states[i + 1].speeds.speedTargetType,
                    },
                    weight: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].weight,
                        states[i + 1].weight,
                    ),
                    config: states[i].config,
                    phase: states[i].phase,
                };
            }
        }

        return {
            distanceFromStart,
            altitude: states[states.length - 1].altitude,
            time: states[states.length - 1].time,
            speeds: {
                calibratedAirspeed: states[states.length - 1].speeds.calibratedAirspeed,
                mach: states[states.length - 1].speeds.mach,
                trueAirspeed: states[states.length - 1].speeds.trueAirspeed,
                groundSpeed: states[states.length - 1].speeds.groundSpeed,
                speedTarget: states[states.length - 1].speeds.speedTarget,
                speedTargetType: states[states.length - 1].speeds.speedTargetType,
            },
            weight: states[states.length - 1].weight,
            config: states[states.length - 1].config,
            phase: states[states.length - 1].phase,
        };
    }
}
