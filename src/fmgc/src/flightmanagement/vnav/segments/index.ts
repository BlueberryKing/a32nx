import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { Common, FlapConf } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { TakeoffSegment } from '@fmgc/flightmanagement/vnav/segments/climb/TakeoffSegment';
import { ClimbSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ClimbSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { CruiseAndDescentSegment } from '@fmgc/flightmanagement/vnav/segments/CruiseAndDescentSegment';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { FmgcFlightPhase } from '@shared/flightphase';
import { McduPseudoWaypointType, SpeedConstraintPrediction } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { HeadwindRepository } from '@fmgc/guidance/vnav/wind/HeadwindRepository';

export enum VerticalSegmentType {
    Unknown = 1,
    Climb = 1 << 1,
    Level = 1 << 2,
    Descent = 1 << 3,
    Accelerate = 1 << 4,
    Decelerate = 1 << 5,
}

export interface Visitor {
    visitBeforeChildren(segment: ProfileSegment, context: VisitorContext): void;
    visitAfterChildren(segment: ProfileSegment, context: VisitorContext): void;
}

export class BuilderVisitor implements Visitor {
    constructor(private builder: ProfileBuilder) {}

    visitBeforeChildren(segment: ProfileSegment): void {
        if (segment.shouldCompute(this.builder.lastState, this.builder)) {
            segment.compute(this.builder.lastState, this.builder);
        }
    }

    visitAfterChildren(segment: ProfileSegment, _context: VisitorContext) {
        if (segment.shouldCompute(this.builder.lastState, this.builder)) {
            segment.onAfterBuildingChildren(this.builder);
        }
    }
}

export interface VisitorContext {
    depth: number;
}

export class McduProfile extends ProfileSegment {
    constructor(context: SegmentContext, constraints: ConstraintReader, stepCoordinator: StepCoordinator) {
        super();

        this.children = [
            new TakeoffSegment(context),
            new ClimbSegment(context, constraints),
            new CruiseAndDescentSegment(context, constraints, stepCoordinator),
        ];
    }

    get repr() {
        return 'McduProfileNode';
    }
}

export interface AircraftConfiguration {
    flapConfig: FlapConf
    speedbrakesExtended: boolean
    gearExtended: boolean
}

export interface AircraftState {
    distanceFromStart: NauticalMiles,
    altitude: Feet,
    time: Seconds,
    speed: Knots,
    trueAirspeed: Knots,
    mach: Mach,
    config: AircraftConfiguration,
    weight: Pounds,
    reason?: string
}

export interface McduPseudoWaypointRequest {
    type: McduPseudoWaypointType,
    state: AircraftState,
    speedConstraint?: SpeedConstraintPrediction
}

export class ProfileBuilder {
    private currentFlightPhase: FmgcFlightPhase;

    private phases: Map<FmgcFlightPhase, AircraftState[]> = new Map();

    mcduPseudoWaypointRequests: McduPseudoWaypointRequest[] = []

    constructor(initialState: AircraftState, phase: FmgcFlightPhase, private buildInReverse: boolean = false) {
        this.currentFlightPhase = phase;

        this.phases.set(phase, [initialState]);
    }

    push(...states: AircraftState[]): ProfileBuilder {
        this.phases.get(this.currentFlightPhase).push(...states);

        return this;
    }

    changePhase(newPhase: FmgcFlightPhase): ProfileBuilder {
        this.currentFlightPhase = newPhase;

        if (!this.phases.has(newPhase)) {
            this.phases.set(newPhase, []);
        }

        return this;
    }

    requestPseudoWaypoint(type: McduPseudoWaypointType, state: AircraftState) {
        this.mcduPseudoWaypointRequests.push({
            type,
            state,
        });
    }

    get lastState(): AircraftState {
        for (let i = this.currentFlightPhase; i >= 0 && i <= 7; i += (this.buildInReverse ? 1 : -1)) {
            const checkpointsOfCurrentPhase = this.phases.get(i);

            if (checkpointsOfCurrentPhase && checkpointsOfCurrentPhase.length > 0) {
                return checkpointsOfCurrentPhase[checkpointsOfCurrentPhase.length - 1];
            }
        }

        return undefined;
    }

    get allCheckpoints(): AircraftState[] {
        const order = [
            FmgcFlightPhase.Preflight,
            FmgcFlightPhase.Takeoff,
            FmgcFlightPhase.Climb,
            FmgcFlightPhase.Cruise,
            FmgcFlightPhase.Descent,
            FmgcFlightPhase.Approach,
            FmgcFlightPhase.GoAround,
            FmgcFlightPhase.Done,
        ];

        return order.reduce((checkpoints, phase) => {
            if (this.phases.has(phase)) {
                checkpoints.push(...this.phases.get(phase));
            }

            return checkpoints;
        }, [] as AircraftState[]);
    }

    get allCheckpointsWithPhase(): AircraftStateWithPhase[] {
        const order = [
            FmgcFlightPhase.Preflight,
            FmgcFlightPhase.Takeoff,
            FmgcFlightPhase.Climb,
            FmgcFlightPhase.Cruise,
            FmgcFlightPhase.Descent,
            FmgcFlightPhase.Approach,
            FmgcFlightPhase.GoAround,
            FmgcFlightPhase.Done,
        ];

        return order.reduce((checkpoints, phase) => {
            if (this.phases.has(phase)) {
                checkpoints.push(...this.phases.get(phase).map((state) => ({ ...state, phase })));
            }

            return checkpoints;
        }, [] as AircraftStateWithPhase[]);
    }

    get currentPhase(): FmgcFlightPhase {
        return this.currentFlightPhase;
    }

    checkpointsOfPhase(phase: FmgcFlightPhase): AircraftState[] {
        if (!this.phases.has(phase)) {
            return [];
        }

        return this.phases.get(phase);
    }

    resetPhaseUpToInitialState(): ProfileBuilder {
        this.phases.get(this.currentFlightPhase).splice(1);

        return this;
    }

    resetPhase(): ProfileBuilder {
        this.phases.set(this.currentFlightPhase, []);

        return this;
    }

    resetPseudoWaypoints(): ProfileBuilder {
        this.mcduPseudoWaypointRequests.length = 0;

        return this;
    }
}

export type AircraftStateWithPhase = AircraftState & { phase: FmgcFlightPhase }

export interface SymbolBuilder {
    push(): void;
}

export class SegmentContext {
    constructor(public atmosphericConditions: AtmosphericConditions, public observer: VerticalProfileComputationParametersObserver, public windRepository: HeadwindRepository) {

    }

    getIsaDeviation() {
        return this.atmosphericConditions.isaDeviation;
    }

    computeCrossoverAltitude(cas: Knots, mach: Mach) {
        return this.atmosphericConditions.crossoverAltitude(cas, mach);
    }
}

export class TemporaryStateSequence {
    private states: AircraftState[];

    constructor(...initialStates: AircraftState[]) {
        this.states = initialStates;
    }

    get last(): AircraftState {
        return this.states[this.states.length - 1];
    }

    get first(): AircraftState {
        return this.states[0];
    }

    get length(): number {
        return this.states.length;
    }

    at(index: number): AircraftState {
        return this.states[index];
    }

    reset() {
        this.states.splice(1);
    }

    push(...states: AircraftState[]) {
        this.states.push(...states);
    }

    reverse(): TemporaryStateSequence {
        return new TemporaryStateSequence(
            ...this.states.slice().reverse(),
        );
    }

    allButFirst(): AircraftState[] {
        return this.states.slice(1);
    }

    interpolateEverythingFromStart(distanceFromStart: NauticalMiles): Partial<AircraftState> {
        if (distanceFromStart <= this.states[0].distanceFromStart) {
            return {
                distanceFromStart,
                altitude: this.states[0].altitude,
                speed: this.states[0].speed,
                mach: this.states[0].mach,
            };
        }

        for (let i = 0; i < this.states.length - 1; i++) {
            if (distanceFromStart > this.states[i].distanceFromStart && distanceFromStart <= this.states[i + 1].distanceFromStart) {
                return {
                    distanceFromStart,
                    altitude: Common.interpolate(
                        distanceFromStart,
                        this.states[i].distanceFromStart,
                        this.states[i + 1].distanceFromStart,
                        this.states[i].altitude,
                        this.states[i + 1].altitude,
                    ),
                    speed: Common.interpolate(
                        distanceFromStart,
                        this.states[i].distanceFromStart,
                        this.states[i + 1].distanceFromStart,
                        this.states[i].speed,
                        this.states[i + 1].speed,
                    ),
                    mach: Common.interpolate(
                        distanceFromStart,
                        this.states[i].distanceFromStart,
                        this.states[i + 1].distanceFromStart,
                        this.states[i].mach,
                        this.states[i + 1].mach,
                    ),
                };
            }
        }

        return {
            distanceFromStart,
            altitude: this.last.altitude,
            speed: this.last.speed,
            mach: this.last.mach,
        };
    }
}
