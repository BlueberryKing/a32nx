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
                time: states[0].time,
                altitude: states[0].altitude,
                weight: states[0].weight,
                speed: states[0].speed,
                mach: states[0].mach,
                trueAirspeed: states[0].trueAirspeed,
                config: states[0].config,
                phase: states[0].phase,
            };
        }

        for (let i = 0; i < states.length - 1; i++) {
            if (distanceFromStart > states[i].distanceFromStart && distanceFromStart <= states[i + 1].distanceFromStart) {
                return {
                    distanceFromStart,
                    time: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].time,
                        states[i + 1].time,
                    ),
                    altitude: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].altitude,
                        states[i + 1].altitude,
                    ),
                    weight: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].weight,
                        states[i + 1].weight,
                    ),
                    speed: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].speed,
                        states[i + 1].speed,
                    ),
                    trueAirspeed: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].trueAirspeed,
                        states[i + 1].trueAirspeed,
                    ),
                    mach: Common.interpolate(
                        distanceFromStart,
                        states[i].distanceFromStart,
                        states[i + 1].distanceFromStart,
                        states[i].mach,
                        states[i + 1].mach,
                    ),
                    config: states[i].config,
                    phase: states[i].phase,
                };
            }
        }

        return {
            distanceFromStart,
            time: states[states.length - 1].time,
            altitude: states[states.length - 1].altitude,
            weight: states[states.length - 1].weight,
            speed: states[states.length - 1].speed,
            trueAirspeed: states[states.length - 1].trueAirspeed,
            mach: states[states.length - 1].mach,
            config: states[states.length - 1].config,
            phase: states[states.length - 1].phase,
        };
    }
}
