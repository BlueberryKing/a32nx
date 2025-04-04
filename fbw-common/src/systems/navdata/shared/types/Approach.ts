import { RunwayDesignator } from './Runway';
import { DatabaseItem, ProcedureTransition } from './Common';
import { ProcedureLeg } from './ProcedureLeg';
import { AirportSubsectionCode, SectionCode } from './SectionCode';

export enum ApproachType {
  Unknown,
  LocBackcourse,
  VorDme,
  Fms,
  Igs,
  Ils,
  Gls,
  Loc,
  Mls,
  Ndb,
  Gps,
  NdbDme,
  Rnav,
  Vortac,
  Tacan,
  Sdf,
  Vor,
  MlsTypeA,
  Lda,
  MlsTypeBC,
}

export enum LevelOfService {
  Lpv = 1 << 0,
  Lpv200 = 1 << 1,
  Lp = 1 << 2,
  Lnav = 1 << 3,
  LnavVnav = 1 << 4,
}

export interface Approach extends DatabaseItem<SectionCode.Airport> {
  subSectionCode: AirportSubsectionCode.ApproachProcedures;

  /** Runway this approach is for. */
  runwayIdent: string;
  /** Runway number this approach is for. */
  runwayNumber: number;
  /** Runway designator this approach is for.  */
  runwayDesignator: RunwayDesignator;

  /** Multiple indicator/suffix. Empty string when none. */
  multipleIndicator: string;

  /**
   * Type of approach guidance
   */
  type: ApproachType;

  /**
   * RNP-AR approach?
   */
  authorisationRequired: boolean;

  /**
   * RNP-AR missed approach?
   */
  missedApproachAuthorisationRequired: boolean;

  /**
   * SBAS level of service authorised bitfield
   * not available for all backends
   */
  levelOfService?: LevelOfService;

  /**
   * Arrival transitions
   */
  transitions: ProcedureTransition[];

  /**
   * Approach legs (common legs and runway transition legs), ending at the MAP
   */
  legs: ProcedureLeg[];

  /**
   * Missed approach legs, starting at the MAP
   */
  missedLegs: ProcedureLeg[];
}
