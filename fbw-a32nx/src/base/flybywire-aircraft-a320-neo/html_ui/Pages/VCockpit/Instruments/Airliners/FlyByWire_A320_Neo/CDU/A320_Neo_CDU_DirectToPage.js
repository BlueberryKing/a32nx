// Copyright (c) 2020, 2022 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

// TODO this whole thing is thales layout...

class CDUDirectToPage {
    static ShowPage(mcdu, directToObject, wptsListIndex = 0, isRadialInPilotEntered = false) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.DirectToPage;
        mcdu.returnPageCallback = () => {
            CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex, isRadialInPilotEntered);
        };

        mcdu.activeSystem = 'FMGC';

        const waypointsCell = ["", "", "", "", ""];
        let iMax = 5;
        let eraseLabel = "";
        let eraseLine = "";
        let insertLabel = "";
        let insertLine = "";
        if (mcdu.flightPlanService.hasTemporary) {
            // Invalid state, should not be able to call up DIR when a temporary exists
            if (!directToObject) {
                mcdu.eraseTemporaryFlightPlan(() => {
                    CDUDirectToPage.ShowPage(mcdu);
                });
                return;
            }

            iMax--;
            eraseLabel = "\xa0DIR TO[color]amber";
            eraseLine = "{ERASE[color]amber";
            insertLabel = "TMPY\xa0[color]amber";
            insertLine = "DIRECT*[color]amber";
            mcdu.onLeftInput[5] = () => {
                mcdu.eraseTemporaryFlightPlan(() => {
                    CDUDirectToPage.ShowPage(mcdu);
                });
            };
            mcdu.onRightInput[5] = () => {
                mcdu.insertTemporaryFlightPlan(() => {
                    SimVar.SetSimVarValue("K:A32NX.FMGC_DIR_TO_TRIGGER", "number", 0);
                    CDUFlightPlanPage.ShowPage(mcdu);
                });
            };
        }

        mcdu.onLeftInput[0] = (value) => {
            if (value === FMCMainDisplay.clrValue) {
                mcdu.eraseTemporaryFlightPlan(() => {
                    CDUDirectToPage.ShowPage(mcdu, undefined, wptsListIndex);
                });
                return;
            }

            Fmgc.WaypointEntryUtils.getOrCreateWaypoint(mcdu, value, false).then((w) => {
                if (w) {
                    mcdu.eraseTemporaryFlightPlan(() => {
                        directToObject = {
                            nonFlightPlanFix: w
                        };

                        mcdu.directTo(directToObject).then(() => {
                            CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
                        }).catch(err => {
                            mcdu.setScratchpadMessage(NXFictionalMessages.internalError);
                            console.error(err);
                        });
                    });
                } else {
                    mcdu.setScratchpadMessage(NXSystemMessages.notInDatabase);
                }
            }).catch((err) => {
                // Rethrow if error is not an FMS message to display
                if (err.type === undefined) {
                    throw err;
                }

                mcdu.showFmsErrorMessage(err.type);
            });
        };

        // DIRECT TO
        mcdu.onRightInput[1] = (s, scratchpadCallback) => {
            if (!directToObject) {
                mcdu.setScratchpadMessage(NXSystemMessages.notAllowed);
                scratchpadCallback();
                return;
            }

            mcdu.eraseTemporaryFlightPlan(() => {
                // TODO delete is really bad
                delete directToObject.withAbeam;
                delete directToObject.courseIn;
                delete directToObject.courseOut;

                mcdu.directTo(directToObject).then(() => {
                    CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
                }).catch(err => {
                    mcdu.setScratchpadMessage(NXFictionalMessages.internalError);
                    console.error(err);
                });
            });
        };

        // ABEAM
        mcdu.onRightInput[2] = () => {
            mcdu.setScratchpadMessage(NXFictionalMessages.notYetImplemented);
        };

        const plan = mcdu.flightPlanService.active;
        const defaultRadialIn = CDUDirectToPage.computeDefaultRadialIn(plan, directToObject);

        // RADIAL IN
        mcdu.onRightInput[3] = (s, scratchpadCallback) => {
            if (!directToObject) {
                mcdu.setScratchpadMessage(NXSystemMessages.notAllowed);
                scratchpadCallback();
                return;
            }

            let course = undefined;
            let isPilotEntered = false;
            if (s === FMCMainDisplay.clrValue) {
                if (directToObject.courseIn !== undefined && defaultRadialIn !== undefined) {
                    mcdu.eraseTemporaryFlightPlan(() => {
                        directToObject.courseIn = defaultRadialIn;

                        mcdu.directTo(directToObject).then(() => {
                            CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
                        }).catch(err => {
                            mcdu.setScratchpadMessage(NXFictionalMessages.internalError);
                            console.error(err);
                        });
                    });
                    return;
                } else {
                    mcdu.setScratchpadMessage(NXSystemMessages.notAllowed);
                    scratchpadCallback();
                    return;
                }
            } else if (s === "" && defaultRadialIn !== undefined) {
                course = defaultRadialIn;
            } else if (/^\d{1,3}/.test(s)) {
                course = parseInt(s);
                if (course > 360) {
                    mcdu.setScratchpadMessage(NXSystemMessages.entryOutOfRange);
                    scratchpadCallback();
                    return;
                }

                isPilotEntered = true;
            } else {
                // TODO this should allow a true course
                mcdu.setScratchpadMessage(NXSystemMessages.formatError);
                scratchpadCallback();
                return;
            }

            mcdu.eraseTemporaryFlightPlan(() => {
                // TODO delete is really bad
                delete directToObject.withAbeam;
                directToObject.courseIn = course % 360;
                delete directToObject.courseOut;

                mcdu.directTo(directToObject).then(() => {
                    CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex, isPilotEntered);
                }).catch(err => {
                    mcdu.setScratchpadMessage(NXFictionalMessages.internalError);
                    console.error(err);
                });
            });
        };

        // RADIAL OUT
        mcdu.onRightInput[4] = (s, scratchpadCallback) => {
            if (!directToObject) {
                mcdu.setScratchpadMessage(NXSystemMessages.notAllowed);
                scratchpadCallback();
                return;
            }

            // TODO this should allow a true course
            if (!/^\d{1,3}/.test(s)) {
                mcdu.setScratchpadMessage(NXSystemMessages.formatError);
                scratchpadCallback();
                return;
            }

            const course = parseInt(s);
            if (course > 360) {
                mcdu.setScratchpadMessage(NXSystemMessages.entryOutOfRange);
                scratchpadCallback();
                return;
            }

            mcdu.eraseTemporaryFlightPlan(() => {
                delete directToObject.withAbeam;
                delete directToObject.courseIn;
                directToObject.courseOut = course % 360;

                mcdu.directTo(directToObject).then(() => {
                    CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
                }).catch(err => {
                    mcdu.setScratchpadMessage(NXFictionalMessages.internalError);
                    console.error(err);
                });
            });
        };

        let directWaypointCell = "";
        if (directToObject) {
            if (directToObject.flightPlanLegIndex !== undefined) {
                // Don't just fetch the leg at the index, since the plan might've sequenced after this page was called up
                const directToLeg = plan.maybeElementAt(directToObject.flightPlanLegIndex);

                if (directToLeg && directToLeg.isDiscontinuity === false) {
                    directWaypointCell = directToLeg.ident;
                }
            } else if (directToObject.nonFlightPlanFix !== undefined) {
                directWaypointCell = directToObject.nonFlightPlanFix.ident;
            }
        }

        let i = 0;
        let cellIter = 0;
        wptsListIndex = Math.max(wptsListIndex, mcdu.flightPlanService.active.activeLegIndex);

        const totalWaypointsCount = plan.firstMissedApproachLegIndex;

        while (i < totalWaypointsCount && i + wptsListIndex < totalWaypointsCount && cellIter < iMax) {
            const legIndex = i + wptsListIndex;
            if (plan.elementAt(legIndex).isDiscontinuity) {
                i++;
                continue;
            }

            const leg = plan.legElementAt(legIndex);

            if (leg) {
                if (!leg.isXF()) {
                    i++;
                    continue;
                }

                waypointsCell[cellIter] = "{" + leg.ident + "[color]cyan";
                if (waypointsCell[cellIter]) {
                    mcdu.onLeftInput[cellIter + 1] = () => {
                        mcdu.eraseTemporaryFlightPlan(() => {
                            directToObject = {
                                flightPlanLegIndex: legIndex
                            };

                            mcdu.directTo(directToObject).then(() => {
                                CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
                            }).catch(err => {
                                mcdu.setScratchpadMessage(NXFictionalMessages.internalError);
                                console.error(err);
                            });
                        });
                    };
                }
            } else {
                waypointsCell[cellIter] = "----";
            }
            i++;
            cellIter++;
        }
        if (cellIter < iMax) {
            waypointsCell[cellIter] = "--END--";
        }
        let up = false;
        let down = false;
        if (wptsListIndex < totalWaypointsCount - 5) {
            mcdu.onUp = () => {
                wptsListIndex++;
                CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
            };
            up = true;
        }
        if (wptsListIndex > 0) {
            mcdu.onDown = () => {
                wptsListIndex--;
                CDUDirectToPage.ShowPage(mcdu, directToObject, wptsListIndex);
            };
            down = true;
        }

        const isWithAbeamSelected = directToObject && directToObject.withAbeam;
        const canSelectWithAbeam = directToObject && !isWithAbeamSelected;

        const isRadialInSelected = directToObject && directToObject.courseIn !== undefined;
        const canSelectRadialIn = directToObject && (directToObject.courseIn !== undefined || defaultRadialIn !== undefined);

        let radialInText = "[ ]°";
        if (isRadialInSelected) {
            radialInText = isRadialInPilotEntered
                ? `${directToObject.courseIn.toFixed(0).padStart(3, '0')}°`
                : `{small}${directToObject.courseIn.toFixed(0).padStart(3, '0')}°{end}`;
        } else if (defaultRadialIn !== undefined) {
            radialInText = `{small}${defaultRadialIn.toFixed(0).padStart(3, '0')}°{end}`;
        }

        const isRadialOutSelected = directToObject && directToObject.courseOut !== undefined;
        const canSelectRadialOut = directToObject && directToObject.courseOut !== undefined;
        const radialOut = isRadialOutSelected
            ? `${directToObject.courseOut.toFixed(0).padStart(3, '0')}°`
            : "[ ]°";

        const isDirectToSelected = directToObject && !isWithAbeamSelected && !isRadialInSelected && !isRadialOutSelected;
        const canSelectDirectTo = directToObject && !isDirectToSelected;

        mcdu.setArrows(up, down, false ,false);
        mcdu.setTemplate([
            ["DIR TO"],
            ["\xa0WAYPOINT", "DIST\xa0", "UTC"],
            ["*[" + (directWaypointCell ? directWaypointCell : "\xa0\xa0\xa0\xa0\xa0") + "][color]cyan", "---", "----"],
            ["\xa0F-PLN WPTS"],
            [waypointsCell[0], `DIRECT TO ${canSelectDirectTo ? "}" : " "}[color]${isDirectToSelected ? "yellow" : "cyan"}`],
            ["", "WITH\xa0"],
            [waypointsCell[1], `ABEAM PTS ${canSelectWithAbeam ? "}" : " "}[color]${isWithAbeamSelected ? "yellow" : "cyan"}[color]inop`],
            ["", "RADIAL IN\xa0"],
            [waypointsCell[2], `${radialInText} ${canSelectRadialIn ? "}" : " "}[color]${isRadialInSelected ? "yellow" : "cyan"}`],
            ["", "RADIAL OUT\xa0"],
            [waypointsCell[3], `${radialOut} ${canSelectRadialOut ? "}" : " "}[color]${isRadialOutSelected ? "yellow" : "cyan"}`],
            [eraseLabel, insertLabel],
            [eraseLine ? eraseLine : waypointsCell[4], insertLine]
        ]);
    }

    /**
     *
     * @param {import("../../../../../../../../../systems/fmgc/src/flightplanning/plans/FlightPlan").FlightPlan} plan
     * @param {import("../../../../../../../../../systems/fmgc/src/flightplanning/types/DirectTo").DirectTo} directToObject
     * @returns {number | undefined}
     */
    static computeDefaultRadialIn(plan, directToObject) {
        if (!directToObject || directToObject.flightPlanLegIndex === undefined) {
            return undefined;
        }

        const directToLeg = plan.maybeElementAt(directToObject.flightPlanLegIndex);
        if (!directToLeg || directToLeg.isDiscontinuity === true || directToLeg.terminationWaypoint() === null) {
            return undefined;
        }

        const maybeLegBefore = plan.maybeElementAt(directToObject.flightPlanLegIndex - 1);
        const maybeLegAfter = plan.maybeElementAt(directToObject.flightPlanLegIndex + 1);

        if (directToObject.flightPlanLegIndex > plan.activeLegIndex && maybeLegBefore && maybeLegBefore.isDiscontinuity === false && maybeLegBefore.terminationWaypoint() !== null) {
            const trueRadialIn = Avionics.Utils.computeGreatCircleHeading(directToLeg.terminationWaypoint().location, maybeLegBefore.terminationWaypoint().location);

            return A32NX_Util.trueToMagnetic(
                trueRadialIn,
                A32NX_Util.getRadialMagVar(directToLeg.terminationWaypoint())
            );
        } else if (maybeLegAfter && maybeLegAfter.isDiscontinuity === false && maybeLegAfter.terminationWaypoint() !== null) {
            const trueRadialIn = 180 + Avionics.Utils.computeGreatCircleHeading(directToLeg.terminationWaypoint().location, maybeLegAfter.terminationWaypoint().location);

            return A32NX_Util.trueToMagnetic(
                trueRadialIn,
                A32NX_Util.getRadialMagVar(directToLeg.terminationWaypoint())
            );
        }

        return undefined;
    }
}
