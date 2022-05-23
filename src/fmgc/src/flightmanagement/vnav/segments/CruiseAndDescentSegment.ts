import { AircraftState, BuilderVisitor, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { CruiseSegment } from '@fmgc/flightmanagement/vnav/segments/cruise/CruiseSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { ManagedDescentSegment } from '@fmgc/flightmanagement/vnav/segments/ManagedDescentSegment';
import { ApproachSegment } from '@fmgc/flightmanagement/vnav/segments/ApproachSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { FmgcFlightPhase } from '@shared/flightphase';

export class CruiseAndDescentSegment extends ProfileSegment {
    private descentSegment: ManagedDescentSegment;

    private approachSegment: ApproachSegment;

    constructor(private context: NodeContext, private constraintReader: ConstraintReader, private stepCoordinator: StepCoordinator) {
        super();

        this.approachSegment = new ApproachSegment(context, constraintReader);
        this.descentSegment = new ManagedDescentSegment(context, constraintReader);
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const initialState = this.getInitialState(this.context, this.constraintReader);
        const temporaryDescentBuilder = new ProfileBuilder(initialState, FmgcFlightPhase.Approach, true);
        const temporaryDescentVisitor = new BuilderVisitor(temporaryDescentBuilder);

        const temporaryCruiseBuilder = new ProfileBuilder(state, FmgcFlightPhase.Cruise);
        const temporaryCruiseVisitor = new BuilderVisitor(temporaryCruiseBuilder);

        for (let i = 0; i < 4; i++) {
            if (temporaryDescentBuilder.currentPhase === FmgcFlightPhase.Descent) {
                temporaryDescentBuilder.resetPhase().changePhase(FmgcFlightPhase.Approach).resetPhaseUpToInitialState();
            }

            temporaryCruiseBuilder.resetPhaseUpToInitialState();

            this.approachSegment.accept(temporaryDescentVisitor);
            this.descentSegment.accept(temporaryDescentVisitor);

            const cruiseSegment = new CruiseSegment(this.context, this.stepCoordinator, state.distanceFromStart, temporaryDescentBuilder.lastState.distanceFromStart);
            cruiseSegment.accept(temporaryCruiseVisitor);

            initialState.weight += temporaryCruiseBuilder.lastState.weight - temporaryDescentBuilder.lastState.weight;
            initialState.time += temporaryCruiseBuilder.lastState.time - temporaryDescentBuilder.lastState.time;
        }

        builder.changePhase(FmgcFlightPhase.Cruise);
        builder.push(...temporaryCruiseBuilder.checkpointsOfPhase(FmgcFlightPhase.Cruise));
        builder.changePhase(FmgcFlightPhase.Descent);
        builder.push(...temporaryDescentBuilder.checkpointsOfPhase(FmgcFlightPhase.Descent).slice().reverse());
        builder.changePhase(FmgcFlightPhase.Approach);
        builder.push(...temporaryDescentBuilder.checkpointsOfPhase(FmgcFlightPhase.Approach).slice().reverse());
    }

    get repr() {
        return 'CruiseAndDescentSegment';
    }

    private getInitialState(context: NodeContext, constraintReader: ConstraintReader): AircraftState {
        const { destinationAirfieldElevation, approachSpeed, zeroFuelWeight, isFlaps3Landing } = context.observer.get();

        return {
            altitude: destinationAirfieldElevation + 50,
            distanceFromStart: constraintReader.totalFlightPlanDistance,
            time: 0,
            weight: zeroFuelWeight + 2500,
            speed: approachSpeed,
            mach: context.atmosphericConditions.computeMachFromCas(destinationAirfieldElevation + 50, approachSpeed),
            trueAirspeed: context.atmosphericConditions.computeTasFromCas(destinationAirfieldElevation + 50, approachSpeed),
            config: {
                flapConfig: isFlaps3Landing ? FlapConf.CONF_3 : FlapConf.CONF_FULL,
                speedbrakesExtended: false,
                gearExtended: true,
            },
        };
    }
}
