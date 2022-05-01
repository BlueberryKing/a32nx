import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { AccelFactorMode, Common } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftConfiguration } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { FlightModel } from '@fmgc/guidance/vnav/FlightModel';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { ClimbSegment } from './ClimbSegment';
import { CruiseAndDescentSegment } from './CruiseAndDescentSegment';
import { TakeoffSegment } from './TakeoffSegment';

enum VerticalSegmentType {
    Unknown = 1,
    Climb = 1 << 1,
    Level = 1 << 2,
    Descent = 1 << 3,
    Accelerate = 1 << 4,
    Decelerate = 1 << 5,
}

interface Visitor {
    visitBeforeChildren(node: ProfileSegment, context: VisitorContext): void;
    visitAfterChildren(node: ProfileSegment, context: VisitorContext): void;
}

export class PrinterVisitor implements Visitor {
    visitBeforeChildren(node: ProfileSegment, context: VisitorContext): void {
        console.log(`${'  '.repeat(context.depth)} - ${node.repr}`);
    }
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

interface VisitorContext {
    depth: number;
}

export abstract class ProfileSegment {
    protected children: ProfileSegment[] = []

    compute(_state: AircraftState, _builder: ProfileBuilder) { }

    accept(visitor: Visitor, context: VisitorContext = { depth: 0 }) {
        visitor.visitBeforeChildren(this, context);

        for (const child of this.children) {
            child.accept(visitor, { depth: context.depth + 1 });
        }

        visitor.visitAfterChildren(this, context);
    }

    getSymbols(_symbolBuilder: any) { }

    get repr(): string {
        return 'Unknown node';
    }

    get type(): VerticalSegmentType {
        return VerticalSegmentType.Unknown;
    }
}

export class McduProfile extends ProfileSegment {
    constructor(context: NodeContext, constraints: ConstraintReader) {
        super();

        this.children = [
            new TakeoffSegment(context),
            new ClimbSegment(context, constraints),
            new CruiseAndDescentSegment(),
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
