import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { Common } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftConfiguration } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { TakeoffSegment } from '@fmgc/flightmanagement/vnav/segments/TakeoffSegment';
import { ClimbSegment } from '@fmgc/flightmanagement/vnav/segments/ClimbSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export enum VerticalSegmentType {
    Unknown = 1,
    Climb = 1 << 1,
    Level = 1 << 2,
    Descent = 1 << 3,
    Accelerate = 1 << 4,
    Decelerate = 1 << 5,
}

export interface Visitor {
    visitBeforeChildren(node: ProfileSegment, context: VisitorContext): void;
    visitAfterChildren(node: ProfileSegment, context: VisitorContext): void;
}

export class PrinterVisitor implements Visitor {
    visitBeforeChildren(node: ProfileSegment, context: VisitorContext): void {
        console.log(`${'  '.repeat(context.depth)} - ${node.repr}`);
    }

    visitAfterChildren(node: ProfileSegment, context: VisitorContext) { }
}

export class BuilderVisitor implements Visitor {
    constructor(private builder: ProfileBuilder) {}

    visitBeforeChildren(node: ProfileSegment): void {
        node.compute(this.builder.lastState, this.builder);
    }

    visitAfterChildren(node: ProfileSegment, context: VisitorContext) { }
}

class SymbolGenerationVisitor implements Visitor {
    constructor(private symbols: SymbolBuilder) { }

    visitBeforeChildren(node: ProfileSegment, _context: VisitorContext): void {
        node.getSymbols(this.symbols);
    }

    visitAfterChildren(_node: ProfileSegment, _context: VisitorContext) { }
}

export interface VisitorContext {
    depth: number;
}

export class McduProfile extends ProfileSegment {
    constructor(context: NodeContext, constraints: ConstraintReader) {
        super();

        this.children = [
            new TakeoffSegment(context),
            new ClimbSegment(context, constraints),
        ];
    }

    get repr() {
        return 'McduProfileNode';
    }
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
}

export class ProfileBuilder {
    private checkpoints: AircraftState[] = [];

    constructor(initialState: AircraftState) {
        this.push(initialState);
    }

    push(state: AircraftState) {
        this.checkpoints.push(state);
    }

    get lastState(): AircraftState {
        return this.checkpoints[this.checkpoints.length - 1];
    }
}

export interface SymbolBuilder {
    push(): void;
}

export class NodeContext {
    constructor(public atmosphericConditions: AtmosphericConditions, public observer: VerticalProfileComputationParametersObserver, public windProfile: HeadwindProfile) {

    }

    getIsaDeviation() {
        return this.atmosphericConditions.isaDeviation;
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
