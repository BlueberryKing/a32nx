import { AircraftState, BuilderVisitor, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { CruiseSegment } from '@fmgc/flightmanagement/vnav/segments/cruise/CruiseSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { ManagedDescentSegment } from '@fmgc/flightmanagement/vnav/segments/descent/ManagedDescentSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AccelFactorMode, FlapConf } from '@fmgc/guidance/vnav/common';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { FmgcFlightPhase } from '@shared/flightphase';
import { CpuTimer, measurePerformance } from '@fmgc/flightmanagement/vnav/common/profiling';
import { ApproachSegment } from '@fmgc/flightmanagement/vnav/segments/approach/ApproachSegment';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';

export class CruiseAndDescentSegment extends ProfileSegment {
    private descentSegment: ManagedDescentSegment;

    private approachSegment: ApproachSegment;

    constructor(private context: SegmentContext, private constraintReader: ConstraintReader, private stepCoordinator: StepCoordinator) {
        super();

        this.approachSegment = new ApproachSegment(context, constraintReader);
        this.descentSegment = new ManagedDescentSegment(context, constraintReader);
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        measurePerformance(() => this.computeInternal(state, builder), (time) => CpuTimer.cruiseAndDescentTime = time);
    }

    private computeInternal(state: AircraftState, builder: ProfileBuilder): void {
        const initialState = this.getInitialState(this.context, this.constraintReader);
        const temporaryDescentBuilder = new ProfileBuilder(initialState, FmgcFlightPhase.Approach, true);
        const temporaryDescentVisitor = new BuilderVisitor(temporaryDescentBuilder);

        const temporaryCruiseBuilder = new ProfileBuilder(state, FmgcFlightPhase.Cruise);
        const temporaryCruiseVisitor = new BuilderVisitor(temporaryCruiseBuilder);

        let weightError = Infinity;
        let timeError = Infinity;

        for (let i = 0; i < 4 && Math.abs(weightError) > 1 && Math.abs(timeError) > 1; i++) {
            if (temporaryDescentBuilder.currentPhase === FmgcFlightPhase.Descent) {
                temporaryDescentBuilder.resetPseudoWaypoints().resetPhase().changePhase(FmgcFlightPhase.Approach).resetPhaseUpToInitialState();
            }

            temporaryCruiseBuilder.resetPhaseUpToInitialState().resetPseudoWaypoints();

            this.approachSegment.accept(temporaryDescentVisitor);
            this.descentSegment.accept(temporaryDescentVisitor);

            const cruiseSegment = new CruiseSegment(this.context, this.stepCoordinator, state.distanceFromStart, temporaryDescentBuilder.lastState.distanceFromStart);
            cruiseSegment.accept(temporaryCruiseVisitor);

            weightError = temporaryCruiseBuilder.lastState.weight - temporaryDescentBuilder.lastState.weight;
            timeError = temporaryCruiseBuilder.lastState.time - temporaryDescentBuilder.lastState.time;

            initialState.weight += weightError;
            initialState.time += timeError;
        }

        builder.changePhase(FmgcFlightPhase.Cruise).push(...temporaryCruiseBuilder.checkpointsOfPhase(FmgcFlightPhase.Cruise));
        builder.mcduPseudoWaypointRequests.push(...temporaryCruiseBuilder.mcduPseudoWaypointRequests);
        builder.ndPseudoWaypointRequests.push(...temporaryCruiseBuilder.ndPseudoWaypointRequests);

        builder.changePhase(FmgcFlightPhase.Descent).push(...temporaryDescentBuilder.checkpointsOfPhase(FmgcFlightPhase.Descent).slice().reverse());

        builder.changePhase(FmgcFlightPhase.Approach).push(...temporaryDescentBuilder.checkpointsOfPhase(FmgcFlightPhase.Approach).slice().reverse());
        builder.mcduPseudoWaypointRequests.push(...temporaryDescentBuilder.mcduPseudoWaypointRequests);
        builder.ndPseudoWaypointRequests.push(...temporaryDescentBuilder.ndPseudoWaypointRequests);
    }

    get repr() {
        return 'CruiseAndDescentSegment';
    }

    private getInitialState(context: SegmentContext, constraintReader: ConstraintReader): AircraftState {
        const { destinationAirfieldElevation, approachSpeed, zeroFuelWeight, isFlaps3Landing } = context.observer.get();

        const headwindAtDestination = context.windRepository.getWindProfile(WindProfileType.Descent)
            .getHeadwindComponent(constraintReader.totalFlightPlanDistance, destinationAirfieldElevation + 50);
        const tasAtDestination = context.atmosphericConditions.computeTasFromCas(destinationAirfieldElevation + 50, approachSpeed);

        return {
            altitude: destinationAirfieldElevation + 50,
            distanceFromStart: constraintReader.totalFlightPlanDistance,
            time: 0,
            speeds: {
                calibratedAirspeed: approachSpeed,
                mach: context.atmosphericConditions.computeMachFromCas(destinationAirfieldElevation + 50, approachSpeed),
                trueAirspeed: tasAtDestination,
                groundSpeed: tasAtDestination - headwindAtDestination.value,
                speedTarget: approachSpeed,
                speedTargetType: AccelFactorMode.CONSTANT_CAS,
            },
            weight: zeroFuelWeight + 6000, // zeroFuelWeight + Initial guess for the amount of fuel on board when landing
            config: {
                flapConfig: isFlaps3Landing ? FlapConf.CONF_3 : FlapConf.CONF_FULL,
                speedbrakesExtended: false,
                gearExtended: true,
            },
        };
    }
}
