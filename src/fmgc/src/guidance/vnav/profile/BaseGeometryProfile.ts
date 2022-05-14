import { Common } from '@fmgc/guidance/vnav/common';
import { PseudoWaypointFlightPlanInfo } from '@fmgc/guidance/PseudoWaypoint';
import { MathUtils } from '@shared/MathUtils';
import { AircraftState } from '@fmgc/flightmanagement/vnav/segments';

export interface PerformancePagePrediction {
    altitude: Feet,
    distanceFromStart: NauticalMiles,
    time: Seconds,
}

export abstract class BaseGeometryProfile {
    public isReadyToDisplay: boolean = false;

    get lastCheckpoint(): AircraftState | null {
        if (this.checkpoints.length < 1) {
            return null;
        }

        return this.checkpoints[this.checkpoints.length - 1];
    }

    constructor(public checkpoints: AircraftState[] = []) { }

    addCheckpointFromLast(checkpointBuilder: (lastCheckpoint: AircraftState) => Partial<AircraftState>) {
        this.checkpoints.push({ ...this.lastCheckpoint, ...checkpointBuilder(this.lastCheckpoint) });
    }

    predictAtTime(time: Seconds): PseudoWaypointFlightPlanInfo {
        const distanceFromStart = this.interpolateDistanceAtTime(time);
        const { altitude, speed } = this.interpolateEverythingFromStart(distanceFromStart);

        return {
            distanceFromStart,
            altitude,
            speed,
            time,
        };
    }

    private interpolateFromCheckpoints<T extends number, U extends number>(
        indexValue: T, keySelector: (checkpoint: AircraftState) => T, valueSelector: (checkpoint: AircraftState) => U,
    ) {
        if (indexValue <= keySelector(this.checkpoints[0])) {
            return valueSelector(this.checkpoints[0]);
        }

        for (let i = 0; i < this.checkpoints.length - 1; i++) {
            if (indexValue > keySelector(this.checkpoints[i]) && indexValue <= keySelector(this.checkpoints[i + 1])) {
                return Common.interpolate(
                    indexValue,
                    keySelector(this.checkpoints[i]),
                    keySelector(this.checkpoints[i + 1]),
                    valueSelector(this.checkpoints[i]),
                    valueSelector(this.checkpoints[i + 1]),
                );
            }
        }

        return valueSelector(this.checkpoints[this.checkpoints.length - 1]);
    }

    private interpolateFromCheckpointsBackwards<T extends number, U extends number>(
        indexValue: T, keySelector: (checkpoint: AircraftState) => T, valueSelector: (checkpoint: AircraftState) => U,
    ) {
        if (indexValue < keySelector(this.checkpoints[this.checkpoints.length - 1])) {
            return valueSelector(this.checkpoints[this.checkpoints.length - 1]);
        }

        for (let i = this.checkpoints.length - 2; i >= 0; i--) {
            if (indexValue <= keySelector(this.checkpoints[i]) && indexValue > keySelector(this.checkpoints[i + 1])) {
                return Common.interpolate(
                    indexValue,
                    keySelector(this.checkpoints[i]),
                    keySelector(this.checkpoints[i + 1]),
                    valueSelector(this.checkpoints[i]),
                    valueSelector(this.checkpoints[i + 1]),
                );
            }
        }

        return valueSelector(this.checkpoints[0]);
    }

    /**
     * Find the time from start at which the profile predicts us to be at a distance along the flightplan.
     * @param distanceFromStart Distance along that path
     * @returns Predicted altitude
     */
    interpolateTimeAtDistance(distanceFromStart: NauticalMiles): Seconds {
        return this.interpolateFromCheckpoints(distanceFromStart, (checkpoint) => checkpoint.distanceFromStart, (checkpoint) => checkpoint.time);
    }

    /**
     * Find the altitude at which the profile predicts us to be at a distance along the flightplan.
     * @param distanceFromStart Distance along that path
     * @returns Predicted altitude
     */
    interpolateAltitudeAtDistance(distanceFromStart: NauticalMiles): Feet {
        return this.interpolateFromCheckpoints(distanceFromStart, (checkpoint) => checkpoint.distanceFromStart, (checkpoint) => checkpoint.altitude);
    }

    /**
     * Find the speed at which the profile predicts us to be at a distance along the flightplan.
     * @param distanceFromStart Distance along that path
     * @returns Predicted speed
     */
    interpolateSpeedAtDistance(distanceFromStart: NauticalMiles): Feet {
        return this.interpolateFromCheckpoints(distanceFromStart, (checkpoint) => checkpoint.distanceFromStart, (checkpoint) => checkpoint.speed);
    }

    /**
     * Find the distanceFromStart at which the profile predicts us to be at a time since departure
     * @param time Time since departure
     * @returns Predicted distance
     */
    interpolateDistanceAtTime(time: Seconds): NauticalMiles {
        return this.interpolateFromCheckpoints(time, (checkpoint) => checkpoint.time, (checkpoint) => checkpoint.distanceFromStart);
    }

    interpolateEverythingFromStart(distanceFromStart: NauticalMiles, doInterpolateAltitude = true): AircraftState {
        if (distanceFromStart <= this.checkpoints[0].distanceFromStart) {
            return {
                distanceFromStart,
                time: this.checkpoints[0].time,
                altitude: this.checkpoints[0].altitude,
                weight: this.checkpoints[0].weight,
                speed: this.checkpoints[0].speed,
                mach: this.checkpoints[0].mach,
                trueAirspeed: this.checkpoints[0].trueAirspeed,
                config: this.checkpoints[0].config,
            };
        }

        for (let i = 0; i < this.checkpoints.length - 1; i++) {
            if (distanceFromStart > this.checkpoints[i].distanceFromStart && distanceFromStart <= this.checkpoints[i + 1].distanceFromStart) {
                return {
                    distanceFromStart,
                    time: Common.interpolate(
                        distanceFromStart,
                        this.checkpoints[i].distanceFromStart,
                        this.checkpoints[i + 1].distanceFromStart,
                        this.checkpoints[i].time,
                        this.checkpoints[i + 1].time,
                    ),
                    altitude: doInterpolateAltitude ? Common.interpolate(
                        distanceFromStart,
                        this.checkpoints[i].distanceFromStart,
                        this.checkpoints[i + 1].distanceFromStart,
                        this.checkpoints[i].altitude,
                        this.checkpoints[i + 1].altitude,
                    ) : this.checkpoints[i].altitude,
                    weight: Common.interpolate(
                        distanceFromStart,
                        this.checkpoints[i].distanceFromStart,
                        this.checkpoints[i + 1].distanceFromStart,
                        this.checkpoints[i].weight,
                        this.checkpoints[i + 1].weight,
                    ),
                    speed: Common.interpolate(
                        distanceFromStart,
                        this.checkpoints[i].distanceFromStart,
                        this.checkpoints[i + 1].distanceFromStart,
                        this.checkpoints[i].speed,
                        this.checkpoints[i + 1].speed,
                    ),
                    trueAirspeed: Common.interpolate(
                        distanceFromStart,
                        this.checkpoints[i].distanceFromStart,
                        this.checkpoints[i + 1].distanceFromStart,
                        this.checkpoints[i].trueAirspeed,
                        this.checkpoints[i + 1].trueAirspeed,
                    ),
                    mach: Common.interpolate(
                        distanceFromStart,
                        this.checkpoints[i].distanceFromStart,
                        this.checkpoints[i + 1].distanceFromStart,
                        this.checkpoints[i].mach,
                        this.checkpoints[i + 1].mach,
                    ),
                    config: this.checkpoints[i].config,
                };
            }
        }

        return {
            distanceFromStart,
            time: this.lastCheckpoint.time,
            altitude: this.lastCheckpoint.altitude,
            weight: this.lastCheckpoint.weight,
            speed: this.lastCheckpoint.speed,
            trueAirspeed: this.lastCheckpoint.trueAirspeed,
            mach: this.lastCheckpoint.mach,
            config: this.lastCheckpoint.config,
        };
    }

    interpolateDistanceAtAltitude(altitude: Feet): NauticalMiles {
        return this.interpolateFromCheckpoints(altitude, (checkpoint) => checkpoint.altitude, (checkpoint) => checkpoint.distanceFromStart);
    }

    interpolateDistanceAtAltitudeBackwards(altitude: Feet): NauticalMiles {
        return this.interpolateFromCheckpointsBackwards(altitude, (checkpoint) => checkpoint.altitude, (checkpoint) => checkpoint.distanceFromStart);
    }

    interpolateFuelAtDistance(distance: NauticalMiles): NauticalMiles {
        return this.interpolateFromCheckpoints(distance, (checkpoint) => checkpoint.distanceFromStart, (checkpoint) => checkpoint.weight);
    }

    interpolatePathAngleAtDistance(distanceFromStart: NauticalMiles): Degrees {
        if (distanceFromStart < this.checkpoints[0].distanceFromStart) {
            return 0;
        }

        for (let i = 0; i < this.checkpoints.length - 1; i++) {
            if (distanceFromStart > this.checkpoints[i].distanceFromStart && distanceFromStart <= this.checkpoints[i + 1].distanceFromStart) {
                return MathUtils.RADIANS_TO_DEGREES * Math.atan(
                    (this.checkpoints[i + 1].altitude - this.checkpoints[i].altitude)
                    / (this.checkpoints[i + 1].distanceFromStart - this.checkpoints[i].distanceFromStart) / 6076.12,
                );
            }
        }

        return 0;
    }

    addCheckpointAtDistanceFromStart(distanceFromStart: NauticalMiles, ...checkpoints: AircraftState[]) {
        if (distanceFromStart <= this.checkpoints[0].distanceFromStart) {
            this.checkpoints.unshift(...checkpoints);

            return;
        }

        for (let i = 0; i < this.checkpoints.length - 1; i++) {
            if (distanceFromStart > this.checkpoints[i].distanceFromStart && distanceFromStart <= this.checkpoints[i + 1].distanceFromStart) {
                this.checkpoints.splice(i + 1, 0, ...checkpoints);

                return;
            }
        }

        this.checkpoints.push(...checkpoints);
    }

    sortCheckpoints() {
        this.checkpoints.sort((a, b) => a.distanceFromStart - b.distanceFromStart);
    }

    finalizeProfile() {
        this.sortCheckpoints();

        this.isReadyToDisplay = true;
    }

    computePredictionToFcuAltitude(fcuAltitude: Feet): PerformancePagePrediction | undefined {
        const maxAltitude = this.checkpoints.reduce((currentMax, checkpoint) => Math.max(currentMax, checkpoint.altitude), 0);

        if (fcuAltitude < this.checkpoints[0].altitude || fcuAltitude > maxAltitude) {
            return undefined;
        }

        const distanceToFcuAltitude = this.interpolateFromCheckpoints(fcuAltitude, (checkpoint) => checkpoint.altitude, (checkpoint) => checkpoint.distanceFromStart);
        const timeToFcuAltitude = this.interpolateTimeAtDistance(distanceToFcuAltitude);

        return {
            altitude: fcuAltitude,
            distanceFromStart: distanceToFcuAltitude,
            time: timeToFcuAltitude,
        };
    }

    getRemainingFuelAtDestination(): Pounds | null {
        if (this.checkpoints.length < 1) {
            return null;
        }

        return this.lastCheckpoint.weight;
    }

    getTimeToDestination(): Pounds | null {
        if (this.checkpoints.length < 1) {
            return null;
        }

        return this.lastCheckpoint.time;
    }
}

type HasAtLeast<T, U extends keyof T> = Pick<T, U> & Partial<Omit<T, U>>

function findLastIndex<T>(array: Array<T>, predicate: (value: T, index: number, obj: T[]) => boolean): number {
    let l = array.length;

    while (l--) {
        if (predicate(array[l], l, array)) {
            return l;
        }
    }

    return -1;
}
