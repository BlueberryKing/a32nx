//  Copyright (c) 2021 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { DescentPathBuilder } from '@fmgc/guidance/vnav/descent/DescentPathBuilder';
import { GuidanceController } from '@fmgc/guidance/GuidanceController';
import { RequestedVerticalMode, TargetAltitude, TargetVerticalSpeed } from '@fmgc/guidance/ControlLaws';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { CoarsePredictions } from '@fmgc/guidance/vnav/CoarsePredictions';
import { FlightPlanManager } from '@fmgc/flightplanning/FlightPlanManager';
import { VerticalMode, ArmedLateralMode, ArmedVerticalMode, isArmed, LateralMode } from '@shared/autopilot';
import { PseudoWaypointFlightPlanInfo } from '@fmgc/guidance/PseudoWaypoint';
import { VerticalProfileComputationParameters, VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { CruisePathBuilder } from '@fmgc/guidance/vnav/cruise/CruisePathBuilder';
import { CruiseToDescentCoordinator } from '@fmgc/guidance/vnav/CruiseToDescentCoordinator';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { McduSpeedProfile, ExpediteSpeedProfile, NdSpeedProfile, ManagedSpeedType } from '@fmgc/guidance/vnav/climb/SpeedProfile';
import { SelectedGeometryProfile } from '@fmgc/guidance/vnav/profile/SelectedGeometryProfile';
import { BaseGeometryProfile, PerfCrzToPrediction } from '@fmgc/guidance/vnav/profile/BaseGeometryProfile';
import { TakeoffPathBuilder } from '@fmgc/guidance/vnav/takeoff/TakeoffPathBuilder';
import { ClimbThrustClimbStrategy, VerticalSpeedStrategy } from '@fmgc/guidance/vnav/climb/ClimbStrategy';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { FmgcFlightPhase } from '@shared/flightphase';
import { TacticalDescentPathBuilder } from '@fmgc/guidance/vnav/descent/TacticalDescentPathBuilder';
import { IdleDescentStrategy } from '@fmgc/guidance/vnav/descent/DescentStrategy';
import { LatchedDescentGuidance } from '@fmgc/guidance/vnav/descent/LatchedDescentGuidance';
import { DescentGuidance } from '@fmgc/guidance/vnav/descent/DescentGuidance';
import { ProfileInterceptCalculator } from '@fmgc/guidance/vnav/descent/ProfileInterceptCalculator';
import { ApproachPathBuilder } from '@fmgc/guidance/vnav/descent/ApproachPathBuilder';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { AircraftToDescentProfileRelation } from '@fmgc/guidance/vnav/descent/AircraftToProfileRelation';
import { WindProfileFactory } from '@fmgc/guidance/vnav/wind/WindProfileFactory';
import { NavHeadingProfile } from '@fmgc/guidance/vnav/wind/AircraftHeadingProfile';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { Leg } from '@fmgc/guidance/lnav/legs/Leg';
import { Geometry } from '../Geometry';
import { GuidanceComponent } from '../GuidanceComponent';
import { NavGeometryProfile, VerticalCheckpointReason, VerticalWaypointPrediction } from './profile/NavGeometryProfile';
import { ClimbPathBuilder } from './climb/ClimbPathBuilder';

export class VnavDriver implements GuidanceComponent {
    version: number = 0;

    private takeoffPathBuilder: TakeoffPathBuilder;

    private climbPathBuilder: ClimbPathBuilder;

    private cruisePathBuilder: CruisePathBuilder;

    private tacticalDescentPathBuilder: TacticalDescentPathBuilder;

    private managedDescentPathBuilder: DescentPathBuilder;

    private approachPathBuilder: ApproachPathBuilder;

    private cruiseToDescentCoordinator: CruiseToDescentCoordinator;

    currentNavGeometryProfile: NavGeometryProfile;

    currentSelectedGeometryProfile?: SelectedGeometryProfile;

    currentNdGeometryProfile?: BaseGeometryProfile;

    private guidanceMode: RequestedVerticalMode;

    private targetVerticalSpeed: TargetVerticalSpeed;

    private targetAltitude: TargetAltitude;

    // eslint-disable-next-line camelcase
    private coarsePredictionsUpdate = new A32NX_Util.UpdateThrottler(5000);

    currentMcduSpeedProfile: McduSpeedProfile;

    timeMarkers = new Map<Seconds, PseudoWaypointFlightPlanInfo | undefined>()

    private constraintReader: ConstraintReader;

    private aircraftToDescentProfileRelation: AircraftToDescentProfileRelation;

    private descentGuidance: DescentGuidance | LatchedDescentGuidance;

    private headingProfile: NavHeadingProfile;

    private isBelowDescentSpeedLimit = false;

    constructor(
        private readonly guidanceController: GuidanceController,
        private readonly computationParametersObserver: VerticalProfileComputationParametersObserver,
        private readonly atmosphericConditions: AtmosphericConditions,
        private readonly windProfileFactory: WindProfileFactory,
        private readonly flightPlanManager: FlightPlanManager,
    ) {
        this.headingProfile = new NavHeadingProfile(flightPlanManager);
        this.currentMcduSpeedProfile = new McduSpeedProfile(this.computationParametersObserver, 0, [], []);

        this.takeoffPathBuilder = new TakeoffPathBuilder(computationParametersObserver, this.atmosphericConditions);
        this.climbPathBuilder = new ClimbPathBuilder(computationParametersObserver, this.atmosphericConditions);
        this.cruisePathBuilder = new CruisePathBuilder(computationParametersObserver, this.atmosphericConditions);
        this.tacticalDescentPathBuilder = new TacticalDescentPathBuilder(this.computationParametersObserver);
        this.managedDescentPathBuilder = new DescentPathBuilder(computationParametersObserver, this.atmosphericConditions);
        this.approachPathBuilder = new ApproachPathBuilder(computationParametersObserver, this.atmosphericConditions);
        this.cruiseToDescentCoordinator = new CruiseToDescentCoordinator(computationParametersObserver, this.cruisePathBuilder, this.managedDescentPathBuilder, this.approachPathBuilder);

        this.constraintReader = new ConstraintReader(guidanceController);

        this.aircraftToDescentProfileRelation = new AircraftToDescentProfileRelation(this.computationParametersObserver);
        this.descentGuidance = VnavConfig.VNAV_USE_LATCHED_DESCENT_MODE
            ? new LatchedDescentGuidance(this.aircraftToDescentProfileRelation, computationParametersObserver, this.atmosphericConditions)
            : new DescentGuidance(this.aircraftToDescentProfileRelation, computationParametersObserver, this.atmosphericConditions);
    }

    init(): void {
        console.log('[FMGC/Guidance] VnavDriver initialized!');
    }

    acceptMultipleLegGeometry(geometry: Geometry) {
        this.recompute(geometry);
    }

    private lastParameters: VerticalProfileComputationParameters = null;

    // Here, we keep a copy of the whatever legs we used to update the descent profile last. We compare it with the legs we get from any new geometries to figure out
    // if the descent profile should be recomputed.
    private oldLegs: Map<number, Leg> = new Map();

    private requestDescentProfileRecomputation: boolean = false;

    update(deltaTime: number): void {
        try {
            const { presentPosition, flightPhase } = this.computationParametersObserver.get();

            if (this.coarsePredictionsUpdate.canUpdate(deltaTime) !== -1) {
                CoarsePredictions.updatePredictions(this.guidanceController, this.atmosphericConditions);
            }

            if (flightPhase >= FmgcFlightPhase.Takeoff) {
                this.constraintReader.updateDistanceToEnd(presentPosition);
                this.updateTimeMarkers();
                this.updateGuidance();
                this.updateDescentSpeedGuidance();
                this.descentGuidance.update(deltaTime, this.constraintReader.distanceToEnd);
            }
        } catch (e) {
            console.error('[FMS] Failed to calculate vertical profile. See exception below.');
            console.error(e);
        }
    }

    recompute(geometry: Geometry): void {
        const newParameters = this.computationParametersObserver.get();

        this.constraintReader.updateGeometry(geometry, newParameters.presentPosition);
        this.windProfileFactory.updateAircraftDistanceFromStart(this.constraintReader.distanceToPresentPosition);
        this.headingProfile.updateGeometry(geometry);
        this.currentMcduSpeedProfile?.update(this.constraintReader.distanceToPresentPosition);

        this.computeVerticalProfileForMcdu(geometry);

        const newLegs = new Map(geometry?.legs ?? []);
        if (this.shouldUpdateDescentProfile(newParameters, newLegs) || this.requestDescentProfileRecomputation) {
            this.oldLegs = new Map(newLegs);
            this.lastParameters = newParameters;
            this.requestDescentProfileRecomputation = false;
            this.isForcingSpeedTargetDownForConstraint = false;

            if (this.currentNavGeometryProfile?.isReadyToDisplay) {
                this.descentGuidance.updateProfile(this.currentNavGeometryProfile);
            }
        }

        this.computeVerticalProfileForNd();
        this.guidanceController.pseudoWaypoints.acceptVerticalProfile();

        this.version++;
    }

    private updateTimeMarkers() {
        if (!this.currentNavGeometryProfile.isReadyToDisplay) {
            return;
        }

        for (const [time] of this.timeMarkers.entries()) {
            const prediction = this.currentNavGeometryProfile.predictAtTime(time);

            this.timeMarkers.set(time, prediction);
        }
    }

    /**
     * Based on the last checkpoint in the profile, we build a profile to the destination
     * @param profile: The profile to build to the destination
     * @param fromFlightPhase: The flight phase to start building the profile from
     */
    private finishProfileInManagedModes(profile: BaseGeometryProfile, fromFlightPhase: FmgcFlightPhase) {
        const { cruiseAltitude } = this.computationParametersObserver.get();

        const managedClimbStrategy = new ClimbThrustClimbStrategy(this.computationParametersObserver, this.atmosphericConditions);
        const stepDescentStrategy = new VerticalSpeedStrategy(this.computationParametersObserver, this.atmosphericConditions, -1000);

        const climbWinds = new HeadwindProfile(this.windProfileFactory.getClimbWinds(), this.headingProfile);
        const cruiseWinds = new HeadwindProfile(this.windProfileFactory.getCruiseWinds(), this.headingProfile);
        const descentWinds = new HeadwindProfile(this.windProfileFactory.getDescentWinds(), this.headingProfile);

        if (fromFlightPhase < FmgcFlightPhase.Climb) {
            this.takeoffPathBuilder.buildTakeoffPath(profile);
        }

        this.currentMcduSpeedProfile = new McduSpeedProfile(
            this.computationParametersObserver,
            this.currentNavGeometryProfile.distanceToPresentPosition,
            this.currentNavGeometryProfile.maxClimbSpeedConstraints,
            this.currentNavGeometryProfile.descentSpeedConstraints,
        );

        if (fromFlightPhase < FmgcFlightPhase.Cruise) {
            this.climbPathBuilder.computeClimbPath(profile, managedClimbStrategy, this.currentMcduSpeedProfile, climbWinds, cruiseAltitude);
        }

        if (profile instanceof NavGeometryProfile && this.cruiseToDescentCoordinator.canCompute(profile)) {
            this.cruiseToDescentCoordinator.buildCruiseAndDescentPath(profile, this.currentMcduSpeedProfile, cruiseWinds, descentWinds, managedClimbStrategy, stepDescentStrategy);

            if (this.currentMcduSpeedProfile.shouldTakeDescentSpeedLimitIntoAccount()) {
                this.cruiseToDescentCoordinator.addSpeedLimitAsCheckpoint(profile);
            }
        }
    }

    private computeVerticalProfileForMcdu(geometry: Geometry) {
        const { flightPhase, presentPosition, fuelOnBoard } = this.computationParametersObserver.get();

        this.currentNavGeometryProfile = new NavGeometryProfile(this.guidanceController, this.constraintReader, this.atmosphericConditions, this.flightPlanManager.getWaypointsCount());

        if (geometry.legs.size <= 0 || !this.computationParametersObserver.canComputeProfile()) {
            return;
        }

        console.time('VNAV computation');
        // TODO: This is where the return to trajectory would go:
        if (flightPhase >= FmgcFlightPhase.Climb) {
            this.currentNavGeometryProfile.addPresentPositionCheckpoint(
                presentPosition,
                fuelOnBoard,
            );
        }

        this.finishProfileInManagedModes(this.currentNavGeometryProfile, Math.max(FmgcFlightPhase.Takeoff, flightPhase));

        this.currentNavGeometryProfile.finalizeProfile();

        console.timeEnd('VNAV computation');

        if (VnavConfig.DEBUG_PROFILE) {
            console.log('this.currentNavGeometryProfile:', this.currentNavGeometryProfile);
            this.currentMcduSpeedProfile.showDebugStats();
        }
    }

    private computeVerticalProfileForNd() {
        const { fcuAltitude, fcuVerticalMode, presentPosition, fuelOnBoard, fcuVerticalSpeed, flightPhase } = this.computationParametersObserver.get();

        this.currentNdGeometryProfile = this.isLatAutoControlActive()
            ? new NavGeometryProfile(this.guidanceController, this.constraintReader, this.atmosphericConditions, this.flightPlanManager.getWaypointsCount())
            : new SelectedGeometryProfile();

        if (!this.computationParametersObserver.canComputeProfile()) {
            return;
        }

        if (flightPhase >= FmgcFlightPhase.Climb) {
            this.currentNdGeometryProfile.addPresentPositionCheckpoint(
                presentPosition,
                fuelOnBoard,
            );
        } else {
            this.takeoffPathBuilder.buildTakeoffPath(this.currentNdGeometryProfile);
        }

        if (!this.shouldObeyAltitudeConstraints()) {
            this.currentNdGeometryProfile.resetAltitudeConstraints();
        }

        const speedProfile = this.shouldObeySpeedConstraints()
            ? this.currentMcduSpeedProfile
            : new NdSpeedProfile(
                this.computationParametersObserver,
                this.currentNdGeometryProfile.distanceToPresentPosition,
                this.currentNdGeometryProfile.maxClimbSpeedConstraints,
                this.currentNdGeometryProfile.descentSpeedConstraints,
            );

        const tacticalClimbModes = [
            VerticalMode.OP_CLB,
        ];

        const tacticalDescentModes = [
            VerticalMode.OP_DES,
            VerticalMode.DES,
        ];

        // Check if we're in the climb or descent phase and compute an appropriate tactical path.
        // A tactical path is a profile in the currently selected modes.
        if (tacticalClimbModes.includes(fcuVerticalMode) || fcuVerticalMode === VerticalMode.VS && fcuVerticalSpeed > 0) {
            const climbStrategy = fcuVerticalMode === VerticalMode.VS
                ? new VerticalSpeedStrategy(this.computationParametersObserver, this.atmosphericConditions, fcuVerticalSpeed)
                : new ClimbThrustClimbStrategy(this.computationParametersObserver, this.atmosphericConditions);

            const climbWinds = new HeadwindProfile(this.windProfileFactory.getClimbWinds(), this.headingProfile);

            this.climbPathBuilder.computeClimbPath(this.currentNdGeometryProfile, climbStrategy, speedProfile, climbWinds, fcuAltitude);
        } else if ((tacticalDescentModes.includes(fcuVerticalMode) || fcuVerticalMode === VerticalMode.VS && fcuVerticalSpeed < 0) && fcuAltitude < presentPosition.alt) {
            // The idea here is that we compute a profile to FCU alt in the current modes, find the intercept with the managed profile. And then compute the managed profile from there.

            const vdev = this.aircraftToDescentProfileRelation.computeLinearDeviation();
            const descentStrategy = this.getAppropriateTacticalDescentStrategy(fcuVerticalMode, fcuVerticalSpeed, vdev);
            const descentWinds = new HeadwindProfile(this.windProfileFactory.getDescentWinds(), this.headingProfile);

            // Build path to FCU altitude.
            this.tacticalDescentPathBuilder.buildTacticalDescentPath(this.currentNdGeometryProfile, descentStrategy, speedProfile, descentWinds, fcuAltitude);

            if (this.isLatAutoControlActive() && this.aircraftToDescentProfileRelation.isValid) {
                // The guidance profile is typically not refreshed after the descent is started, whereas the tactical profile is recomputed constantly,
                // so the `distanceFromStart`s are very different in the profiles.
                const guidanceDistanceFromStart = this.aircraftToDescentProfileRelation.distanceFromStart;
                const tacticalDistanceFromStart = this.currentNdGeometryProfile.checkpoints[0].distanceFromStart;
                const offset = tacticalDistanceFromStart - guidanceDistanceFromStart;

                // Compute intercept between managed profile and tactical path.
                const interceptDistance = ProfileInterceptCalculator.calculateIntercept(
                    this.currentNdGeometryProfile.checkpoints,
                    this.aircraftToDescentProfileRelation.currentProfile.checkpoints,
                    offset,
                );

                if (interceptDistance) {
                    const isManagedDescent = fcuVerticalMode === VerticalMode.DES;
                    const reason = isManagedDescent ? VerticalCheckpointReason.InterceptDescentProfileManaged : VerticalCheckpointReason.InterceptDescentProfileSelected;
                    const interceptCheckpoint = this.currentNdGeometryProfile.addInterpolatedCheckpoint(interceptDistance, { reason });

                    const isTooCloseToDrawIntercept = Math.abs(vdev) < 100;
                    const isInterceptOnlyAtFcuAlt = interceptCheckpoint.altitude - fcuAltitude < 100;

                    // In managed mode, we remove all checkpoints after (maybe including) the intercept
                    // In selected mode, we keep the checkpoints after the intercept
                    const interceptIndex = this.currentNdGeometryProfile.checkpoints.findIndex((checkpoint) => checkpoint.reason === reason);
                    if (isManagedDescent) {
                        this.currentNdGeometryProfile.checkpoints.splice(
                            isTooCloseToDrawIntercept || isInterceptOnlyAtFcuAlt ? interceptIndex : interceptIndex + 1,
                            this.currentNdGeometryProfile.checkpoints.length,
                        );
                    } else if (isTooCloseToDrawIntercept || isInterceptOnlyAtFcuAlt) {
                        this.currentNdGeometryProfile.checkpoints.splice(
                            isTooCloseToDrawIntercept || isInterceptOnlyAtFcuAlt ? interceptIndex : interceptIndex + 1,
                            1,
                        );
                    }
                }
            }
        }

        // After computing the tactical profile, we wish to finish it up in managed modes.
        if (this.isLatAutoControlActive() && this.currentNdGeometryProfile) {
            this.currentNdGeometryProfile.checkpoints.push(
                ...this.currentNavGeometryProfile.checkpoints
                    .slice()
                    .filter(({ distanceFromStart }) => distanceFromStart > this.currentNdGeometryProfile.lastCheckpoint?.distanceFromStart ?? 0),
            );

            if ((tacticalDescentModes.includes(fcuVerticalMode) || fcuVerticalMode === VerticalMode.VS && fcuVerticalSpeed < 0) && fcuAltitude < presentPosition.alt) {
                // It's possible that the level off arrow was spliced from the profile because we spliced it at the intercept, so we need to add it back.
                const levelOffArrow = this.currentNdGeometryProfile.findVerticalCheckpoint(VerticalCheckpointReason.CrossingFcuAltitudeDescent);
                let levelOffDistance = levelOffArrow?.distanceFromStart ?? 0;

                if (fcuVerticalMode === VerticalMode.DES && !levelOffArrow) {
                    levelOffDistance = this.currentNdGeometryProfile.interpolateDistanceAtAltitudeBackwards(fcuAltitude, true);
                    this.currentNdGeometryProfile.addInterpolatedCheckpoint(levelOffDistance, { reason: VerticalCheckpointReason.CrossingFcuAltitudeDescent });
                }

                for (let i = 1; i < this.currentNdGeometryProfile.checkpoints.length; i++) {
                    const checkpoint = this.currentNdGeometryProfile.checkpoints[i];
                    if (checkpoint.distanceFromStart < levelOffDistance || checkpoint.altitude - fcuAltitude >= -10) {
                        continue;
                    }

                    const previousCheckpoint = this.currentNdGeometryProfile.checkpoints[i - 1];

                    if (previousCheckpoint.reason !== VerticalCheckpointReason.PresentPosition && Math.round(previousCheckpoint.altitude) > Math.round(checkpoint.altitude)) {
                        this.currentNdGeometryProfile.addInterpolatedCheckpoint(
                            Math.max(levelOffDistance, previousCheckpoint.distanceFromStart),
                            { reason: VerticalCheckpointReason.ContinueDescent },
                        );

                        break;
                    }
                }
            }
        }

        this.currentNdGeometryProfile.finalizeProfile();

        if (VnavConfig.DEBUG_PROFILE) {
            console.log('this.currentNdGeometryProfile:', this.currentNdGeometryProfile);
        }
    }

    private getAppropriateTacticalDescentStrategy(fcuVerticalMode: VerticalMode, fcuVerticalSpeed: FeetPerMinute, vdev: Feet) {
        if (fcuVerticalMode === VerticalMode.VS && fcuVerticalSpeed < 0) {
            return new VerticalSpeedStrategy(this.computationParametersObserver, this.atmosphericConditions, fcuVerticalSpeed);
        }

        if (this.aircraftToDescentProfileRelation.isValid && fcuVerticalMode === VerticalMode.DES) {
            if (vdev > 0 && this.aircraftToDescentProfileRelation.isPastTopOfDescent()) {
                return new IdleDescentStrategy(
                    this.computationParametersObserver,
                    this.atmosphericConditions,
                    { flapConfig: FlapConf.CLEAN, gearExtended: false, speedbrakesExtended: fcuVerticalMode === VerticalMode.DES },
                );
            }

            return new VerticalSpeedStrategy(
                this.computationParametersObserver,
                this.atmosphericConditions,
                this.aircraftToDescentProfileRelation.isAboveSpeedLimitAltitude() ? -1000 : -500,
            );

            // TODO: Implement prediction in the case of being below profile, but within the geometric path
            // In that case, we should use a constant path angle, but I don't have a DescentStrategy for this yet.
        }

        return new IdleDescentStrategy(
            this.computationParametersObserver,
            this.atmosphericConditions,
        );
    }

    private shouldObeySpeedConstraints(): boolean {
        const { fcuSpeed } = this.computationParametersObserver.get();

        // TODO: Take MACH into account
        return this.isLatAutoControlActive() && fcuSpeed <= 0;
    }

    shouldObeyAltitudeConstraints(): boolean {
        const { fcuArmedLateralMode, fcuArmedVerticalMode, fcuVerticalMode } = this.computationParametersObserver.get();

        const verticalModesToApplyAltitudeConstraintsFor = [
            VerticalMode.CLB,
            VerticalMode.ALT,
            VerticalMode.ALT_CPT,
            VerticalMode.ALT_CST_CPT,
            VerticalMode.ALT_CST,
            VerticalMode.DES,
        ];

        return isArmed(fcuArmedVerticalMode, ArmedVerticalMode.CLB)
            || isArmed(fcuArmedLateralMode, ArmedLateralMode.NAV)
            || verticalModesToApplyAltitudeConstraintsFor.includes(fcuVerticalMode);
    }

    computeVerticalProfileForExpediteClimb(): SelectedGeometryProfile | undefined {
        try {
            const { fcuAltitude, presentPosition, fuelOnBoard } = this.computationParametersObserver.get();

            const greenDotSpeed = Simplane.getGreenDotSpeed();
            if (!greenDotSpeed) {
                return undefined;
            }

            const selectedSpeedProfile = new ExpediteSpeedProfile(greenDotSpeed);
            const expediteGeometryProfile = new SelectedGeometryProfile();
            const climbStrategy = new ClimbThrustClimbStrategy(this.computationParametersObserver, this.atmosphericConditions);
            const climbWinds = new HeadwindProfile(this.windProfileFactory.getClimbWinds(), this.headingProfile);

            expediteGeometryProfile.addPresentPositionCheckpoint(presentPosition, fuelOnBoard);
            this.climbPathBuilder.computeClimbPath(expediteGeometryProfile, climbStrategy, selectedSpeedProfile, climbWinds, fcuAltitude);

            expediteGeometryProfile.finalizeProfile();

            if (VnavConfig.DEBUG_PROFILE) {
                console.log(expediteGeometryProfile);
            }

            return expediteGeometryProfile;
        } catch (e) {
            console.error(e);
            return new SelectedGeometryProfile();
        }
    }

    isLatAutoControlActive(): boolean {
        const { fcuLateralMode } = this.computationParametersObserver.get();

        return fcuLateralMode === LateralMode.NAV
            || fcuLateralMode === LateralMode.LOC_CPT
            || fcuLateralMode === LateralMode.LOC_TRACK
            || fcuLateralMode === LateralMode.RWY;
    }

    isSelectedVerticalModeActive(): boolean {
        const { fcuVerticalMode } = this.computationParametersObserver.get();
        const isExpediteModeActive = SimVar.GetSimVarValue('L:A32NX_FMA_EXPEDITE_MODE', 'number') === 1;

        return isExpediteModeActive
            || fcuVerticalMode === VerticalMode.VS
            || fcuVerticalMode === VerticalMode.FPA
            || fcuVerticalMode === VerticalMode.OP_CLB
            || fcuVerticalMode === VerticalMode.OP_DES;
    }

    private isForcingSpeedTargetDownForConstraint: boolean = false;

    private updateDescentSpeedGuidance() {
        const { flightPhase, managedDescentSpeed, managedDescentSpeedMach, presentPosition, descentSpeedLimit } = this.computationParametersObserver.get();
        const isExpediteModeActive = SimVar.GetSimVarValue('L:A32NX_FMA_EXPEDITE_MODE', 'number') === 1;
        const speedControlManual = Simplane.getAutoPilotAirspeedSelected();
        const isHoldActive = this.guidanceController.isManualHoldActive();
        const currentDistanceFromStart = this.constraintReader.distanceToPresentPosition;
        const currentAltitude = presentPosition.alt;

        if (flightPhase !== FmgcFlightPhase.Descent || speedControlManual || isExpediteModeActive || isHoldActive) {
            return;
        }

        // Use min() just for safety
        let newSpeedTarget = Math.min(managedDescentSpeed, this.currentMcduSpeedProfile.getTarget(currentDistanceFromStart, currentAltitude, ManagedSpeedType.Descent));

        // TODO: This is a mess... This code takes care of decelerating before a constraint / speed limit to reach the target speed at the desired point.
        // What makes this complicated is that we don't want to the speed target to jump around during the deceleration segment, only because the profile calculation computes
        // that deceleration is only necessary a bit later. Once we start the deceleration to a point, we go through with it.
        // This means, we need to save the state "in deceleration segment" and latch it. But we need to figure out when to reset this value again, as not to get stuck in a bad state.
        if (Number.isFinite(this.tacticalDescentPathBuilder?.nextDecelerationAltitude?.altitude)
            && Number.isFinite(this.tacticalDescentPathBuilder?.nextDecelerationAltitude?.speed)
            && (currentAltitude < this.tacticalDescentPathBuilder.nextDecelerationAltitude.altitude || this.isBelowDescentSpeedLimit)
        ) {
            newSpeedTarget = Math.min(newSpeedTarget, this.tacticalDescentPathBuilder.nextDecelerationAltitude.speed);
            this.isBelowDescentSpeedLimit = true;
        } else if (this.currentMcduSpeedProfile.shouldTakeDescentSpeedLimitIntoAccount()) {
            // Hystersis so the speed target doesn't flicker while flying at the speed limit altitude. I don't know if this is in the real aircraft,
            // but it definitely make a lot of sense.
            if (currentAltitude < descentSpeedLimit.underAltitude) {
                this.isBelowDescentSpeedLimit = true;
            } else if (currentAltitude > descentSpeedLimit.underAltitude + 100) {
                this.isBelowDescentSpeedLimit = false;
            }
        }

        if (Number.isFinite(this.tacticalDescentPathBuilder?.nextDecelerationToSpeedConstraint?.distanceFromStart)
            && Number.isFinite(this.tacticalDescentPathBuilder?.nextDecelerationToSpeedConstraint?.speed)
        ) {
            if (this.isForcingSpeedTargetDownForConstraint
                || !this.isForcingSpeedTargetDownForConstraint && currentDistanceFromStart > this.tacticalDescentPathBuilder.nextDecelerationToSpeedConstraint.distanceFromStart
            ) {
                newSpeedTarget = Math.min(newSpeedTarget, this.tacticalDescentPathBuilder.nextDecelerationToSpeedConstraint.speed);
                this.isForcingSpeedTargetDownForConstraint = true;
            } else if (newSpeedTarget < this.tacticalDescentPathBuilder?.nextDecelerationToSpeedConstraint?.speed) {
                this.isForcingSpeedTargetDownForConstraint = false;
            }
        }

        const econMachAsCas = SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', managedDescentSpeedMach);

        SimVar.SetSimVarValue('L:A32NX_SPEEDS_MANAGED_PFD', 'knots', Math.min(newSpeedTarget, econMachAsCas));
    }

    private updateGuidance(): void {
        let newGuidanceMode = RequestedVerticalMode.None;
        let newVerticalSpeed = 0;
        let newAltitude = 0;

        if (this.guidanceController.isManualHoldActive()) {
            const fcuVerticalMode = SimVar.GetSimVarValue('L:A32NX_FMA_VERTICAL_MODE', 'Enum');
            if (fcuVerticalMode === VerticalMode.DES) {
                const holdSpeed = SimVar.GetSimVarValue('L:A32NX_FM_HOLD_SPEED', 'number');
                const atHoldSpeed = this.atmosphericConditions.currentAirspeed <= (holdSpeed + 5);
                if (atHoldSpeed) {
                    newGuidanceMode = RequestedVerticalMode.VsSpeed;
                    newVerticalSpeed = -1000;
                    newAltitude = 0;
                }
            }
        }

        if (this.guidanceController.isManualHoldActive() || this.guidanceController.isManualHoldNext()) {
            let holdSpeedCas = SimVar.GetSimVarValue('L:A32NX_FM_HOLD_SPEED', 'number');
            const holdDecelReached = SimVar.GetSimVarValue('L:A32NX_FM_HOLD_DECEL', 'bool');

            const speedControlManual = Simplane.getAutoPilotAirspeedSelected();
            const isMach = Simplane.getAutoPilotMachModeActive();
            if (speedControlManual && holdDecelReached) {
                if (isMach) {
                    const holdValue = Simplane.getAutoPilotMachHoldValue();
                    holdSpeedCas = this.atmosphericConditions.computeCasFromMach(this.atmosphericConditions.currentAltitude, holdValue);
                } else {
                    holdSpeedCas = Simplane.getAutoPilotAirspeedHoldValue();
                }
            }

            const holdSpeedTas = this.atmosphericConditions.computeTasFromCas(this.atmosphericConditions.currentAltitude, holdSpeedCas);

            this.guidanceController.setHoldSpeed(holdSpeedTas);
        }

        if (newGuidanceMode !== this.guidanceMode) {
            this.guidanceMode = newGuidanceMode;
            SimVar.SetSimVarValue('L:A32NX_FG_REQUESTED_VERTICAL_MODE', 'number', this.guidanceMode);
        }
        if (newVerticalSpeed !== this.targetVerticalSpeed) {
            this.targetVerticalSpeed = newVerticalSpeed;
            SimVar.SetSimVarValue('L:A32NX_FG_TARGET_VERTICAL_SPEED', 'number', this.targetVerticalSpeed);
        }
        if (newAltitude !== this.targetAltitude) {
            this.targetAltitude = newAltitude;
            SimVar.SetSimVarValue('L:A32NX_FG_TARGET_ALTITUDE', 'number', this.targetAltitude);
        }
    }

    getLinearDeviation(): Feet | null {
        if (!this.aircraftToDescentProfileRelation.isValid) {
            return null;
        }

        return this.aircraftToDescentProfileRelation.computeLinearDeviation();
    }

    private shouldUpdateDescentProfile(newParameters: VerticalProfileComputationParameters, newLegs: Map<number, Leg>): boolean {
        // While in the descent phase, we don't want to update the profile anymore
        if (this.lastParameters === null) {
            return true;
        }

        return newParameters.flightPhase < FmgcFlightPhase.Descent || newParameters.flightPhase > FmgcFlightPhase.Approach
            || (!this.flightPlanManager.isCurrentFlightPlanTemporary() && this.didLegsChange(this.oldLegs, newLegs))
            || numberOrNanChanged(this.lastParameters.cruiseAltitude, newParameters.cruiseAltitude)
            || numberOrNanChanged(this.lastParameters.managedDescentSpeed, newParameters.managedDescentSpeed)
            || numberOrNanChanged(this.lastParameters.managedDescentSpeedMach, newParameters.managedDescentSpeedMach)
            || numberOrNanChanged(this.lastParameters.approachQnh, newParameters.approachQnh)
            || numberOrNanChanged(this.lastParameters.approachTemperature, newParameters.approachTemperature);
    }

    private didLegsChange(oldLegs: Map<number, Leg>, newLegs: Map<number, Leg>): boolean {
        for (const [index, legA] of newLegs) {
            const legB = oldLegs.get(index);

            if (index < this.guidanceController.activeLegIndex) {
                continue;
            }

            if (!legA?.ident !== !legB?.ident) {
                return true;
            }
        }

        return false;
    }

    public getDestinationPrediction(): VerticalWaypointPrediction | null {
        return this.currentNavGeometryProfile?.waypointPredictions?.get(this.flightPlanManager.getDestinationIndex());
    }

    public getPerfCrzToPrediction(): PerfCrzToPrediction | null {
        if (!this.currentNavGeometryProfile?.isReadyToDisplay) {
            return null;
        }

        const todOrStep = this.currentNavGeometryProfile.findVerticalCheckpoint(
            VerticalCheckpointReason.StepClimb, VerticalCheckpointReason.StepDescent, VerticalCheckpointReason.TopOfDescent,
        );

        if (!todOrStep) {
            return null;
        }

        return {
            reason: todOrStep.reason,
            distanceFromPresentPosition: todOrStep.distanceFromStart - this.constraintReader.distanceToPresentPosition,
            secondsFromPresent: todOrStep.secondsFromPresent,
        };
    }

    public get distanceToEnd(): NauticalMiles {
        return this.constraintReader.distanceToEnd;
    }

    public findNextSpeedChange(): NauticalMiles | null {
        const { presentPosition, flightPhase } = this.computationParametersObserver.get();

        if (!Simplane.getAutoPilotAirspeedManaged() || SimVar.GetSimVarValue('L:A32NX_FMA_EXPEDITE_MODE', 'number') === 1 || flightPhase === FmgcFlightPhase.Approach) {
            return null;
        }

        let speedTargetType: ManagedSpeedType = ManagedSpeedType.Climb;
        if (flightPhase === FmgcFlightPhase.Cruise) {
            speedTargetType = ManagedSpeedType.Cruise;
        } else if (flightPhase === FmgcFlightPhase.Descent) {
            speedTargetType = ManagedSpeedType.Descent;
        }

        const distanceToPresentPosition = this.constraintReader.distanceToPresentPosition;

        // We don't want to show the speed change dot at acceleration altiude, so we have to make sure the speed target is econ speed, not SRS speed.
        const speedTarget = flightPhase < FmgcFlightPhase.Climb
            ? this.currentMcduSpeedProfile.getTarget(distanceToPresentPosition, presentPosition.alt, ManagedSpeedType.Climb)
            : SimVar.GetSimVarValue('L:A32NX_SPEEDS_MANAGED_PFD', 'knots');

        for (let i = 1; i < this.currentNdGeometryProfile.checkpoints.length - 1; i++) {
            const checkpoint = this.currentNdGeometryProfile.checkpoints[i];

            if (checkpoint.distanceFromStart < distanceToPresentPosition) {
                continue;
            } else if (checkpoint.reason === VerticalCheckpointReason.TopOfClimb || checkpoint.reason === VerticalCheckpointReason.TopOfDescent) {
                return null;
            }

            if (Math.min(checkpoint.speed, this.atmosphericConditions.computeCasFromMach(checkpoint.altitude, checkpoint.mach))
                - Math.max(this.currentNdGeometryProfile.checkpoints[i - 1].speed, speedTarget) > 1) {
                // Candiate for a climb speed change
                if (speedTargetType === ManagedSpeedType.Climb) {
                    return this.currentNdGeometryProfile.checkpoints[i - 1].distanceFromStart;
                }
            } else if (checkpoint.speed - Math.min(speedTarget, this.currentNdGeometryProfile.checkpoints[i - 1].speed) < -1) {
                if (speedTargetType === ManagedSpeedType.Descent) {
                    return this.currentNdGeometryProfile.checkpoints[i - 1].distanceFromStart;
                }
            }
        }

        return null;
    }

    public invalidateFlightPlanProfile(): void {
        this.requestDescentProfileRecomputation = true;

        if (this.currentNavGeometryProfile) {
            this.currentNavGeometryProfile.invalidate();
        }
    }
}

/// To check whether the value changed from old to new, but not if both values are NaN. (NaN !== NaN in JS)
export function numberOrNanChanged(oldValue: number, newValue: number): boolean {
    return Number.isNaN(oldValue) !== Number.isNaN(newValue) && oldValue !== newValue;
}
