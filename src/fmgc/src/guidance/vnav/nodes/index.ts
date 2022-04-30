import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { AccelFactorMode, Common, FlapConf } from '@fmgc/guidance/vnav/common';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftConfiguration } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { FlightModel } from '@fmgc/guidance/vnav/FlightModel';
import { MaxAltitudeConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { VerticalProfileComputationParameters, VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';

interface Visitor {
    visit(node: ProfileNode, context: VisitorContext): void;
}

export class PrinterVisitor implements Visitor {
    visit(node: ProfileNode, context: VisitorContext): void {
        console.log(`${'  '.repeat(context.depth)} - ${node.repr}`);
    }
}

export class BuilderVisitor implements Visitor {
    constructor(private builder: ProfileBuilder) {}

    visit(node: ProfileNode): void {
        node.compute(this.builder.lastState, this.builder);
    }
}

interface VisitorContext {
    depth: number;
}

abstract class ProfileNode {
    protected children: ProfileNode[] = []

    compute(_state: AircraftState, _builder: ProfileBuilder) { }

    accept(visitor: Visitor, context: VisitorContext = { depth: 0 }) {
        visitor.visit(this, context);

        for (const child of this.children) {
            child.accept(visitor, { depth: context.depth + 1 });
        }
    }

    get repr() {
        return 'Generic Node';
    }
}

export class McduProfileNode extends ProfileNode {
    constructor(context: NodeContext, constraints: ConstraintReader) {
        super();

        this.children = [
            new TakeoffNode(context),
            new ClimbNode(context, constraints),
            new CruiseAndDescentNode(),
        ];
    }

    get repr() {
        return 'McduProfileNode';
    }
}

class TakeoffNode extends ProfileNode {
    /**
     *
     */
    constructor(context: NodeContext) {
        super();

        const { thrustReductionAltitude, accelerationAltitude } = context.observer.get();

        this.children = [
            new PureClimbToAltitudeNode(context, new TakeoffThrustSetting(context.atmosphericConditions), thrustReductionAltitude),
            new PureClimbToAltitudeNode(context, new ClimbThrustSetting(context.atmosphericConditions), accelerationAltitude),
            new ConfigurationChangeNode(context, { flapConfig: FlapConf.CLEAN, speedbrakesExtended: false, gearExtended: false }),
        ];
    }

    get repr() {
        return 'TakeoffNode';
    }
}

class ClimbNode extends ProfileNode {
    constructor(context: NodeContext, constraints: ConstraintReader) {
        super();

        const { cruiseAltitude, climbSpeedLimit, managedClimbSpeed, managedClimbSpeedMach } = context.observer.get();
        const crossoverAltitude = 30000; // TODO

        this.children = [
            new ManagedClimbNode(context, climbSpeedLimit.speed, climbSpeedLimit.underAltitude, constraints),
            new ManagedClimbNode(context, managedClimbSpeed, crossoverAltitude, constraints),
            new ManagedClimbMachNode(cruiseAltitude, managedClimbSpeedMach),
        ];
    }

    get repr() {
        return 'ClimbNode';
    }
}

class ConfigurationChangeNode extends ProfileNode {
    constructor(_context: NodeContext, private config: AircraftConfiguration) {
        super();
    }

    override compute(state: AircraftState, builder: ProfileBuilder): void {
        builder.push({
            ...state,
            config: this.config,
        });
    }
}

class ManagedClimbNode extends ProfileNode {
    constructor(context: NodeContext, maxSpeed: Knots, toAltitude: Feet, constraints: ConstraintReader) {
        super();

        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);

        this.children.push(
            new PureClimbToAltitudeNode(context, climbThrust, toAltitude, Infinity),
        );

        const allConstraints = [...constraints.climbAlitudeConstraints, ...constraints.climbSpeedConstraints];
        allConstraints.sort((a, b) => b.distanceFromStart - a.distanceFromStart);

        let currentMaxSpeed = maxSpeed;
        for (const constraint of allConstraints) {
            if ('maxSpeed' in constraint) {
                this.children.push(new PureAccelerationNode(currentMaxSpeed));
                currentMaxSpeed = Math.min(currentMaxSpeed, constraint.maxSpeed);
            } else {
                this.children.push(new ClimbToAltConstraint(context, climbThrust, constraint));
            }
        }

        this.children.push(
            new PureAccelerationNode(currentMaxSpeed),
        );

        this.children = this.children.reverse();
    }

    get repr() {
        return 'ManagedClimbNode';
    }
}

class ManagedClimbMachNode extends ProfileNode {
    constructor(private toAltitude: Feet, private maxSpeed: Mach) {
        super();
    }

    get repr() {
        return 'ManagedClimbMachNode';
    }
}

class CruiseAndDescentNode extends ProfileNode {
    get repr() {
        return 'CruiseAndDescentNode';
    }
}

class PureClimbToAltitudeNode extends ProfileNode {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    constructor(
        private context: NodeContext, private thrustSetting: ThrustSetting, toAltitude: Feet, maxDistance: NauticalMiles = Infinity,
    ) {
        super();

        this.endConditions = [
            (state) => state.altitude > toAltitude,
            (state) => state.distanceFromStart > maxDistance,
        ];
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const endState = this.integrator.integrate(state,
            this.endConditions,
            constantThrustPropagator(this.thrustSetting, this.context.atmosphericConditions, this.context.observer.get()));

        builder.push(endState);
    }

    get repr() {
        return 'PureClimbToAltitudeNode';
    }
}

class ClimbToAltConstraint extends ProfileNode {
    constructor(context: NodeContext, thrustSetting: ThrustSetting, constraint: MaxAltitudeConstraint) {
        super();

        this.children = [
            new PureClimbToAltitudeNode(context, thrustSetting, constraint.maxAltitude, constraint.distanceFromStart),
            new PureLevelSegmentNode(context, constraint.distanceFromStart),
        ];
    }

    get repr() {
        return 'ClimbToAltConstraint';
    }
}

class PureAccelerationNode extends ProfileNode {
    constructor(private toSpeed: Knots) {
        super();
    }

    get repr() {
        return 'PureAccelerationNode';
    }
}

class PureLevelSegmentNode extends ProfileNode {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    constructor(
        private context: NodeContext, maxDistance: NauticalMiles,
    ) {
        super();

        this.endConditions = [
            (state) => state.distanceFromStart > maxDistance,
        ];
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const pitchTarget = new FlightPathAnglePitchTarget(0);

        const endState = this.integrator.integrate(
            state,
            this.endConditions,
            constantPitchPropagator(pitchTarget, this.context.atmosphericConditions, this.context.observer.get()),
        );

        builder.push(endState);
    }

    get repr() {
        return 'PureLevelSegmentNode';
    }
}

interface ThrustSetting {
    getThrustAndFuelFlow(state: AircraftState): [number, number]
}

class ClimbThrustSetting implements ThrustSetting {
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

class TakeoffThrustSetting implements ThrustSetting {
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

type IntegrationEndCondition = (state: AircraftState) => boolean;
type IntegrationPropagator = (state: AircraftState) => AircraftState;

class Integrator {
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

interface NodeContext {
    atmosphericConditions: AtmosphericConditions,
    observer: VerticalProfileComputationParametersObserver,
}

// What if I have two propagators?
// 1. pitch to thrust
// 2. thrust to pitch
//

function constantThrustPropagator(thrustSetting: ThrustSetting, atmosphericConditions: AtmosphericConditions, { tropoPause }: VerticalProfileComputationParameters) {
    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const theta = Common.getTheta(state.altitude, atmosphericConditions.isaDeviation, state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);

        const headwind = 0; // TODO
        const tas = Common.CAStoTAS(state.speed, theta, delta);
        const groundSpeed = tas - headwind;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            atmosphericConditions.isaDeviation,
            state.altitude > tropoPause,
            AccelFactorMode.CONSTANT_CAS,
        );

        const [thrust, fuelFlow] = thrustSetting.getThrustAndFuelFlow(state);
        const distance: NauticalMiles = 0.1;
        const pathAngle: Radians = FlightModel.getConstantThrustPathAngle(thrust, state.weight, drag, accelerationFactor);
        const verticalSpeed: FeetPerMinute = 101.268 * tas * Math.sin(pathAngle);
        const stepTime: Seconds = 3600 * distance / groundSpeed;
        const fuelBurned: Pounds = fuelFlow * stepTime / 3600;

        return {
            distanceFromStart: state.distanceFromStart + distance,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: state.speed,
            trueAirspeed: state.trueAirspeed,
            mach: state.mach,
            config: state.config,
        };
    };
}

function constantPitchPropagator(pitch: PitchTarget, atmosphericConditions: AtmosphericConditions, { tropoPause }: VerticalProfileComputationParameters) {
    return (state: AircraftState): AircraftState => {
        const delta = Common.getDelta(state.altitude, state.altitude > tropoPause);
        const theta = Common.getTheta(state.altitude, atmosphericConditions.isaDeviation, state.altitude > tropoPause);
        const drag = FlightModel.getDrag(state.weight, state.mach, delta, state.config.speedbrakesExtended, state.config.gearExtended, state.config.flapConfig);
        const delta2 = Common.getDelta2(delta, state.mach);
        const theta2 = Common.getTheta2(theta, state.mach);

        const headwind = 0; // TODO
        const tas = Common.CAStoTAS(state.speed, theta, delta);
        const groundSpeed = tas - headwind;

        const accelerationFactor = Common.getAccelerationFactor(
            state.mach,
            state.altitude,
            atmosphericConditions.isaDeviation,
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

        return {
            distanceFromStart: state.distanceFromStart + distance,
            altitude: state.altitude + stepTime / 60 * verticalSpeed,
            time: state.time + stepTime,
            weight: state.weight - fuelBurned,
            speed: state.speed,
            trueAirspeed: state.trueAirspeed,
            mach: state.mach,
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

class FlightPathAnglePitchTarget implements PitchTarget {
    constructor(private flightPathAngle: Degrees) { }

    getPathAngle(_: AircraftState): Radians {
        return this.flightPathAngle;
    }
}
