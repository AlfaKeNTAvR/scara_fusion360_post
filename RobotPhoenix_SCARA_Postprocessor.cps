description = "Robot Phoenix SCARA Postprocessor";
vendor = "BNRobotics";
legal = "MIT License";
extension = "xml";

setCodePage("ascii");

capabilities = CAPABILITY_MILLING | CAPABILITY_JET;

allowedCircularPlanes = 0; // Allow any circular motion.

const maxSpeed = 0.5;

let cmdID = 0;
let glueEnabled = false;

let lastPosition = { x: 0, y: 0, z: 0 };
let lastSpeed = 0.1; // Initialize with homing speed.

let xyzFormat = createFormat({decimals: 2, trim: true});
let feedFormat = createFormat({decimals: 2, trim: true});

let programDraft = []; // Store commands to write later.

// RZ constants:
const EPS = 1e-6;

// Configure machine's zero-angle direction here.
// Example: (0, -1) => 0° when pointing along -Y.
const ZERO_DIR = { x: 0, y: -1 };

// Positive angle multiplier (flip to -1 if your RZ positive is opposite
// expectation):
const ROT_SIGN = 1;

// Fixed mechanical/tool-frame offset in degrees if your tool frame is rotated:
const TOOL_OFFSET_DEG = 0;

// Max allowed excursion from the 0° reference to avoid cable windup
const ANGLE_SPAN_LIMIT = 360 * 2;


// User-defined properties:
properties = {
    zTableOffset: {
        title: "Z Table Offset (mm)",
        // description: "Negative distance to the table surface in mm.",
        group: "1 - General",
        type: "number",
        value: -95,
        range: [-180, 0],
        scope: "post",
    },
    rapidMotionSpeed: {
        title: "Rapid Motion Speed (fraction)",
        // description: "The robot speed fraction for fast movement without glue application.",
        group: "1 - General",
        type: "number",
        value: 0.3,
        range: [0.01, maxSpeed],
        scope: "post",
    },
    enableRZ: {
        title: "Enable RZ Orientation",
        group: "1 - General",
        type: "boolean",
        value: false,
        scope: "post",
    },
    defaultBlendingRadius: {
        title: "Default Blending Radius (mm)",
        // description: "The robot speed fraction for fast movement without glue application.",
        group: "2 - Advanced",
        type: "number",
        value: 10,
        // range: [0, maxVelocityFraction],
        scope: "post",
    },
    enableCornerPrerotate: {
        title: "Pre-Rotate at Long Segments",
        group: "2 - Advanced",
        type: "boolean",
        value: true,
        scope: "post",
    },
    minStraightForPrerotateMM: {
        title: "Pre-Rotate Min Length (mm)",
        group: "2 - Advanced",
        type: "number",
        value: 30,   // pick what “long” means for you (e.g., 20–50 mm)
        range: [0, 1000],
        scope: "post",
    },
};

function onOpen() {
    writeln(
`<?xml version="1.0" encoding="UTF-8"?>
<Cmds>
    <YiKingSoft>2.9.8(RU3)</YiKingSoft>
    <VarList>
        <Var>
            <VarID>0</VarID>
            <strVartype>bool</strVartype>
            <VarName>enable_vacuum</VarName>
            <SVarValue>false</SVarValue>
            <bInitMaxAndMin>false</bInitMaxAndMin>
            <MaxValue>true</MaxValue>
            <MinValue>false</MinValue>
            <bExternOper>false</bExternOper>
            <Comment></Comment>
        </Var>
        <Var>
            <VarID>1</VarID>
            <strVartype>double</strVartype>
            <VarName>robot_speed</VarName>
            <SVarValue>0.1000</SVarValue>
            <bInitMaxAndMin>true</bInitMaxAndMin>
            <MaxValue>0.5000</MaxValue>
            <MinValue>0.0500</MinValue>
            <bExternOper>false</bExternOper>
            <Comment></Comment>
        </Var>
        <Var>
            <VarID>2</VarID>
            <strVartype>bool</strVartype>
            <VarName>enable_glue</VarName>
            <SVarValue>false</SVarValue>
            <bInitMaxAndMin>false</bInitMaxAndMin>
            <MaxValue>true</MaxValue>
            <MinValue>false</MinValue>
            <bExternOper>false</bExternOper>
            <Comment></Comment>
        </Var>
    </VarList>
    <PointList/>
    <ArrayList/>
    <Cmd ItemText="Stop_Subprogram">
        <Cmdinfo>
            <CmdSetType>ECST_Segment</CmdSetType>
            <CmdName>Subprogram</CmdName>
            <ShowName>Stop_Subprogram</ShowName>
            <CmdId>` + (cmdID++) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="Call Initialize">
                <Cmdinfo>
                    <CmdSetType>ECST_Logic</CmdSetType>
                    <CmdName>Call Subprogram</CmdName>
                    <ShowName>Call Initialize</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDSubprogram>Initialize Subprogram</Param0-IDSubprogram>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="Subprogram End"/>
        </ChildCmd>
    </Cmd>
    <Cmd ItemText="Initialize Subprogram">
        <Cmdinfo>
            <CmdSetType>ECST_Segment</CmdSetType>
            <CmdName>Subprogram</CmdName>
            <ShowName>Initialize Subprogram</ShowName>
            <CmdId>` + (cmdID++) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetRobot">
                <Cmdinfo>
                    <CmdSetType>ECST_Segment</CmdSetType>
                    <CmdName>SetRobot</CmdName>
                    <ShowName>SetRobot</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDRobotNo>1</Param0-IDRobotNo>
                    <Param1-IDRobotType>ERT_Scara</Param1-IDRobotType>
                    <Param2-IDRobotModel>Python450-B6</Param2-IDRobotModel>
                </Cmdinfo>
                <ChildCmd>
                    <Cmd ItemText="enable_glue:=false">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_glue:=false</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_glue:=false </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>
                    <Cmd ItemText="enable_vacuum:=false">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_vacuum:=false</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_vacuum:=false </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>
                    <Cmd ItemText="SetHomingSpeed">
                        <Cmdinfo>
                            <CmdSetType>ECST_Set</CmdSetType>
                            <CmdName>SetFixedSpeed</CmdName>
                            <ShowName>SetHomingSpeed</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDFixedSpeed>true</Param0-IDFixedSpeed>
                            <Param1-IDSpeedRatio>0.1000</Param1-IDSpeedRatio>
                        </Cmdinfo>
                    </Cmd>
                    <Cmd ItemText="MoveDoorHome">
                        <Cmdinfo>
                            <CmdSetType>ECST_Move</CmdSetType>
                            <CmdName>MoveDoor</CmdName>
                            <ShowName>MoveDoorHome</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDTCP>0</Param0-IDTCP>
                            <Param1-IDBase>0</Param1-IDBase>
                            <Param2-IDVelocity>1.0</Param2-IDVelocity>
                            <Param3-IDAcc>1.0</Param3-IDAcc>
                            <Param4-IDJerk>1.0</Param4-IDJerk>
                            <Param5-IDUpVelocity>1.0</Param5-IDUpVelocity>
                            <Param6-IDUpAcc>1.0</Param6-IDUpAcc>
                            <Param7-IDUpJerk>1.0</Param7-IDUpJerk>
                            <Param8-IDDownVelocity>1.0</Param8-IDDownVelocity>
                            <Param9-IDDownAcc>1.0</Param9-IDDownAcc>
                            <Param10-IDDownJerk>1.0</Param10-IDDownJerk>
                            <Param11-IDRotVelocity>1.0</Param11-IDRotVelocity>
                            <Param12-IDRotAcc>1.0</Param12-IDRotAcc>
                            <Param13-IDRotJerk>1.0</Param13-IDRotJerk>
                            <Param14-IDModeset>0</Param14-IDModeset>
                            <Param15-IDUseVel>0.25</Param15-IDUseVel>
                            <Param16-IDTime>1</Param16-IDTime>
                            <Param100-IDTabPointSet>''</Param100-IDTabPointSet>
                            <Param101-IDPointset>1</Param101-IDPointset>
                            <Param102-IDPointId>0</Param102-IDPointId>
                            <Param103-IDPoint>-1</Param103-IDPoint>
                            <Param104-IDX>350</Param104-IDX>
                            <Param105-IDY>0</Param105-IDY>
                            <Param106-IDZ>0</Param106-IDZ>
                            <Param109-IDRZ>0</Param109-IDRZ>
                            <Param110-IDE1>0</Param110-IDE1>
                            <Param111-IDE2>0</Param111-IDE2>
                            <Param112-IDE3>0</Param112-IDE3>
                            <Param113-IDE4>0</Param113-IDE4>
                            <Param114-IDE5>0</Param114-IDE5>
                            <Param115-IDE6>0</Param115-IDE6>
                            <Param128-IDDifferPose>2</Param128-IDDifferPose>
                            <Param150-IDpath>''</Param150-IDpath>
                            <Param151-IDMidSegmentMode>0</Param151-IDMidSegmentMode>
                            <Param152-IDUpDistance>240</Param152-IDUpDistance>
                            <Param153-IDDownDistance>240</Param153-IDDownDistance>
                            <Param158-IDUpDistanceLimit>0</Param158-IDUpDistanceLimit>
                            <Param159-IDDownDistanceLimit>0</Param159-IDDownDistanceLimit>
                            <Param162-IDUpBlendR>1</Param162-IDUpBlendR>
                            <Param163-IDDownBlendR>1</Param163-IDDownBlendR>
                            <Param164-IDRotPercent>0.8</Param164-IDRotPercent>
                        </Cmdinfo>
                    </Cmd>
                    <Cmd ItemText="SetRobot End"/>
                </ChildCmd>
            </Cmd>
            <Cmd ItemText="Subprogram End"/>
        </ChildCmd>
    </Cmd>
    <Cmd ItemText="Main Program">
        <Cmdinfo>
            <CmdSetType>ECST_Segment</CmdSetType>
            <CmdName>Main Program</CmdName>
            <ShowName>Main Program</ShowName>
            <CmdId>` + (cmdID++) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="Call Initialize">
                <Cmdinfo>
                    <CmdSetType>ECST_Logic</CmdSetType>
                    <CmdName>Call Subprogram</CmdName>
                    <ShowName>Call Initialize</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDSubprogram>Initialize Subprogram</Param0-IDSubprogram>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="SetRobot">
                <Cmdinfo>
                    <CmdSetType>ECST_Segment</CmdSetType>
                    <CmdName>SetRobot</CmdName>
                    <ShowName>SetRobot</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDRobotNo>1</Param0-IDRobotNo>
                    <Param1-IDRobotType>ERT_Scara</Param1-IDRobotType>
                    <Param2-IDRobotModel>Python450-B6</Param2-IDRobotModel>
                </Cmdinfo>
                <ChildCmd>
                    <Cmd ItemText="Loop   true">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Loop</CmdName>
                            <ShowName>Loop</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>0</SameTypeID>
                            <PmExp0-IDTimes>true </PmExp0-IDTimes>
                        </Cmdinfo>
                        <ChildCmd>`
  );
}

function onClose() {

    const enableRZ  = getProperty("enableRZ");

    let rzByIndex = new Map();

    if (enableRZ) {
        // 1) Compute RZ on the ORIGINAL sequence
        rzByIndex = computeOutwardYawForGlueMoveL(programDraft);

        // 2) Stamp those RZ values directly onto the glue points (prevents index shift bugs)
        assignGlueRZToPoints(programDraft, rzByIndex);

        // 3) Keep approach/exit rapids aligned (writes p.rz on those rapid points)
        applyApproachExitOrientation_AllRuns(programDraft, rzByIndex, 2);

        // 4) Optional: insert pre-rotate zero-length points for long straights
        if (getProperty("enableCornerPrerotate")) {
            const minLen = Math.max(0, Number(getProperty("minStraightForPrerotateMM")) || 0);
            programDraft = applyCornerPreRotateLongSegments(programDraft, rzByIndex, minLen, xyzFormat);
        }

        // NOTE: We do NOT recompute rzByIndex now; inserted pre-rotate points already carry correct RZ.
    }

    for (var i = 0; i < programDraft.length; i++) {
        let blendRadius = 0.0;
        const blendRadiusSafetyFactor = 0.9; 

        switch (programDraft[i].function) {
            case "writeComment":
                writeComment(programDraft[i].arguments[0]);

                break;

            case "writeGlue":
                writeGlue(programDraft[i].arguments[0]);
                
                break;

            case "writeSpeed":
                writeSpeed(programDraft[i].arguments[0]);
                
                break;

            case "writeVacuum":
                writeVacuum(programDraft[i].arguments[0]);
                
                break;

            case "writePause":
                writePause();
                
                break;

            case "writeDelay": 
                writeDelay(programDraft[i].arguments[0]);
            
                break;

            case "writeMoveL":
                if (programDraft[i + 1].function == "writeMoveC"){
                    const lineLength = programDraft[i].arguments[2];
                    const arcRadius = programDraft[i+1].arguments[2];
                    const halfChordLength = programDraft[i+1].arguments[3] / 2;
                                    
                    // Blending is limited by the spatial room available to
                    // place a circular transition.
                    const shortestLength = Math.min(lineLength, 
                                                    halfChordLength, 
                                                    arcRadius);
                    blendRadius = xyzFormat.format(blendRadiusSafetyFactor * shortestLength);

                    // writeln("<!-- ");
                    // writeln("Arc radius: " + arcRadius);
                    // writeln("Half chord length: " + halfChordLength);
                    // writeln("Line length: " + lineLength);
                    // writeln("Shortest length: " + shortestLength);
                    // writeln("Blend radius: " + blendRadius);
                    // writeln("-->");
                }

                else if (programDraft[i + 1].function == "writeMoveL" 
                         && programDraft[i + 1].arguments[1] == "feed") {
                    const lineLength = programDraft[i].arguments[2];
                    const nextLineLength = programDraft[i+1].arguments[2];
                    const defaultBlendingRadius = getProperty("defaultBlendingRadius");

                    blendRadius = xyzFormat.format(Math.min(lineLength / 2, 
                                                            nextLineLength / 2,
                                                            defaultBlendingRadius));
                }

                if (blendRadius < 1){
                    blendRadius = 0;
                }

                // RZ is already stamped onto points earlier when enabled.
                // When disabled, force zero on everything for safety.
                const p1 = programDraft[i].arguments[0];
                if (!enableRZ) {
                    p1.rz = 0;
                }
                // (If enableRZ is true and p1.rz was never set—for example on some rapids—it's fine;
                // writeMoveL fallback already prints 0 when p1.rz is null.)
                    
                writeMoveL(programDraft[i].arguments[0], 
                           blendRadius);
                
                break;

            case "writeMoveC":
                if (programDraft[i+1].function == "writeMoveL") {
                    const lineLength = programDraft[i+1].arguments[2];
                    const arcRadius = programDraft[i].arguments[2];
                    const halfChordLength = programDraft[i].arguments[3] / 2;

                    // Blending is limited by the spatial room available to
                    // place a circular transition.
                    const shortestLength = Math.min(lineLength, 
                                                    halfChordLength, 
                                                    arcRadius);
                    blendRadius = xyzFormat.format(blendRadiusSafetyFactor * shortestLength); 
                }
                
                writeMoveC(programDraft[i].arguments[0], 
                           programDraft[i].arguments[1], 
                           blendRadius);
    
                break;
        }
    }

    writeln(
`                           <Cmd ItemText="Loop End"/>
                        </ChildCmd>
                    </Cmd>
                    <Cmd ItemText="SetRobot End"/>
                </ChildCmd>
            </Cmd>
            <Cmd ItemText="Program End"/>
        </ChildCmd>
    </Cmd>
    <Cmd ItemText="Glue Control Thread">
        <Cmdinfo>
            <CmdSetType>ECST_Segment</CmdSetType>
            <CmdName>Thread</CmdName>
            <ShowName>Glue Control Thread</ShowName>
            <CmdId>` + (cmdID++) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetGlueOutput">
                <Cmdinfo>
                    <CmdSetType>ECST_IO</CmdSetType>
                    <CmdName>SetOutput</CmdName>
                    <ShowName>SetGlueOutput</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDChannel>0</Param0-IDChannel>
                    <PmVar1-IDLevel>2</PmVar1-IDLevel>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="Thread End"/>
        </ChildCmd>
    </Cmd>
    <Cmd ItemText="Vacuum Table Control Thread">
        <Cmdinfo>
            <CmdSetType>ECST_Segment</CmdSetType>
            <CmdName>Thread</CmdName>
            <ShowName>Vacuum Table Control Thread</ShowName>
            <CmdId>` + (cmdID++) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetVacuumTableOutput">
                <Cmdinfo>
                    <CmdSetType>ECST_IO</CmdSetType>
                    <CmdName>SetOutput</CmdName>
                    <ShowName>SetVacuumTableOutput</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDChannel>1</Param0-IDChannel>
                    <PmVar1-IDLevel>0</PmVar1-IDLevel>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="Thread End"/>
        </ChildCmd>
    </Cmd>
    <Cmd ItemText="Speed Control Thread">
        <Cmdinfo>
            <CmdSetType>ECST_Segment</CmdSetType>
            <CmdName>Thread</CmdName>
            <ShowName>Speed Control Thread</ShowName>
            <CmdId>` + (cmdID++) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetOperationSpeed">
                <Cmdinfo>
                    <CmdSetType>ECST_Set</CmdSetType>
                    <CmdName>SetFixedSpeed</CmdName>
                    <ShowName>SetOperationSpeed</ShowName>
                    <CmdId>` + (cmdID++) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDFixedSpeed>true</Param0-IDFixedSpeed>
                    <PmVar1-IDSpeedRatio>1</PmVar1-IDSpeedRatio>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="Thread End"/>
        </ChildCmd>
    </Cmd>
</Cmds>`
    );
}

function onParameter(name, value) {
    switch (name) {
        case "action":
            let sText1 = String(value).toUpperCase();
            let sText2 = new Array();
            sText2 = sText1.split(" ");

        switch (sText2[0]) {
            case "VACUUM":
            if (sText2[1] == "ON") {
                // writeVacuum(true);
                programDraft.push({function: "writeVacuum", arguments: [true]});
            } 

            else if (sText2[1] == "OFF") {
                // writeVacuum(false);
                programDraft.push({function: "writeVacuum", arguments: [false]});
            }
            
            break;

        case "PAUSE":
            // writePause();
            programDraft.push({function: "writePause", arguments: []});
            break;

        case "DELAY":
            const delayTime = parseFloat(sText2[1]);
            // writeDelay(delayTime);
            programDraft.push({function: "writeDelay", arguments: [delayTime]});
            break;

        case "MOVE":
            const rapidMotionSpeed = getProperty("rapidMotionSpeed");
            if (Math.abs(lastSpeed - rapidMotionSpeed) > 0.01) {    
                // writeSpeed(rapidMotionSpeed);
                programDraft.push({function: "writeSpeed", arguments: [rapidMotionSpeed]});
                lastSpeed = rapidMotionSpeed;
            }

            const zOffset = getProperty("zTableOffset");
            const p1 = {x: xyzFormat.format(parseFloat(sText2[1])), 
                        y: xyzFormat.format(parseFloat(sText2[2])), 
                        z: xyzFormat.format(parseFloat(sText2[3])) + zOffset};
            // writeMoveL(p1);
            programDraft.push({function: "writeMoveL", arguments: [p1, "rapid"]});
            lastPosition = { x: p1.x, y: p1.y, z: p1.z };
            break;
        }
    }
}

function onRapid(__x, __y, __z) {
    if (glueEnabled == true) {
        // writeGlue(false);
        programDraft.push({function: "writeGlue", arguments: [false]});
        glueEnabled = false;
    }

    const rapidMotionSpeed = getProperty("rapidMotionSpeed");
    if (Math.abs(lastSpeed - rapidMotionSpeed) > 0.01) {    
        // writeSpeed(rapidMotionSpeed);
        programDraft.push({function: "writeSpeed", arguments: [rapidMotionSpeed]});
        lastSpeed = rapidMotionSpeed;
    }

    const zOffset = getProperty("zTableOffset");
    const p1 = {x: parseFloat(xyzFormat.format(__x)), 
                y: parseFloat(xyzFormat.format(__y)), 
                z: parseFloat(xyzFormat.format(__z)) + zOffset};
    // writeMoveL(p1);
    programDraft.push({function: "writeMoveL", arguments: [p1, "rapid"]});
    lastPosition = { x: p1.x, y: p1.y, z: p1.z };
}

function onLinear(__x, __y, __z, __feed) {
    if (glueEnabled == false) {
        // writeGlue(true);
        programDraft.push({function: "writeGlue", arguments: [true]});
        glueEnabled = true;
    }

    const feed = parseFloat(feedFormat.format(__feed));
    if (Math.abs(lastSpeed - feed) > 0.01) {    
        // writeSpeed(feed);
        programDraft.push({function: "writeSpeed", arguments: [feed]});
        lastSpeed = feed;
    }

    const zOffset = getProperty("zTableOffset");
    const p0 = {x: parseFloat(xyzFormat.format(lastPosition.x)), 
                y: parseFloat(xyzFormat.format(lastPosition.y)), 
                z: parseFloat(xyzFormat.format(lastPosition.z))}; 
    const p1 = {x: parseFloat(xyzFormat.format(__x)), 
                y: parseFloat(xyzFormat.format(__y)), 
                z: parseFloat(xyzFormat.format(__z)) + zOffset};
    const length = Math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2);
    // writeMoveL(p1);
    programDraft.push({function: "writeMoveL", arguments: [p1, "feed", length]});
    lastPosition = { x: p1.x, y: p1.y, z: p1.z };
}

function onCircular(__clockwise, ___cx, __cy, __cz, __x, __y, __z, __feed) {
    const zOffset = getProperty("zTableOffset");
    const start = getCurrentPosition();

    const p0 = { x: parseFloat(xyzFormat.format(start.x)), 
                 y: parseFloat(xyzFormat.format(start.y)), 
                 z: parseFloat(xyzFormat.format(start.z)) + zOffset};

    const center = {x: parseFloat(xyzFormat.format(___cx)), 
                    y: parseFloat(xyzFormat.format(__cy)), 
                    z: parseFloat(xyzFormat.format(__cz)) + zOffset};
    const radius = Math.sqrt((p0.x - center.x) ** 2 + (p0.y - center.y) ** 2);

    let p1 = { x: 0, y: 0, z: 0 };
    const p2 = { x: parseFloat(xyzFormat.format(__x)), 
                y: parseFloat(xyzFormat.format(__y)), 
                z: parseFloat(xyzFormat.format(__z)) + zOffset };

    if (!isFullCircle()) {
        // Calculate the mid-point of the circular arc:
        // 1. Convert start and end points to vectors relative to the arc
        //    center.
        const v0 = { x: p0.x - center.x, y: p0.y - center.y };
        const v2 = { x: p2.x - center.x, y: p2.y - center.y };

        // 2. Compute angles (angle0 and angle2) from the center to each point
        //    using atan2.
        const angle0 = Math.atan2(v0.y, v0.x);
        const angle2 = Math.atan2(v2.y, v2.x);

        // 3. Determine the sweep angle (difference between angle2 and angle0),
        //    adjusted for direction.
        let sweep = angle2 - angle0;
        if (__clockwise && sweep > 0) sweep -= 2 * Math.PI;
        if (!__clockwise && sweep < 0) sweep += 2 * Math.PI;

        // 4. Compute the mid-angle as halfway between angle0 and angle2.
        const midAngle = angle0 + sweep / 2;

        // 5. Use polar-to-Cartesian conversion (cos/sin) with the radius and
        //    mid-angle to get the XY midpoint. Set Z as the average of start
        //    and end Z values.
        p1 = { x: xyzFormat.format(center.x + radius * Math.cos(midAngle)), 
               y: xyzFormat.format(center.y + radius * Math.sin(midAngle)), 
               z: xyzFormat.format((p0.z + p2.z) / 2 )};
    }

    // TODO: Handle full circle case.

    if (glueEnabled == false) {
        // writeGlue(true);
        programDraft.push({function: "writeGlue", arguments: [true]});
        glueEnabled = true;
    }

    const feed = parseFloat(feedFormat.format(__feed));
    if (Math.abs(lastSpeed - feed) > 0.01) {    
        // writeSpeed(feed);
        programDraft.push({function: "writeSpeed", arguments: [feed]});
        lastSpeed = feed;
    }

    // writeMoveC(p1, p2);
    const chordLength = getCircularChordLength();
    programDraft.push({function: "writeMoveC", 
                       arguments: [p1, p2, radius, chordLength]});
    lastPosition = { x: p2.x, y: p2.y, z: p2.z };
}

function onSection() {
    const currentSection = getSection(getCurrentSectionId());
    if (currentSection.hasParameter("operation-comment")) {
        const comment = currentSection.getParameter("operation-comment");
        if (comment) {
            // writeComment("; Start: " + comment);
            programDraft.push({function: "writeComment", arguments: ["; Start: " + comment]});
        }  
    }     
}

function onSectionEnd() {
    const currentSection = getSection(getCurrentSectionId());
    if (currentSection.hasParameter("operation-comment")) {
        const comment = currentSection.getParameter("operation-comment");
        if (comment) {
            // writeComment("; End: " + comment);
            programDraft.push({function: "writeComment", arguments: ["; End: " + comment]});
        }  
    }     
}

// Helper functions:
function writeComment(comment) {
    writeln(
`                    <Cmd ItemText="` + (comment) +`">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Label</CmdName>
                            <ShowName>` + (comment) +`</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function writeGlue(enable){
    writeln(
`                   <Cmd ItemText="enable_glue:=` + (enable ? "true" : "false") + `">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_glue:=` + (enable ? "true" : "false") + `</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_glue:=` + (enable ? "true" : "false") + `</PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function writeSpeed(speed) {
    writeln(
`                   <Cmd ItemText="robot_speed:=` + (speed) + `">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>robot_speed:=` + (speed) + `</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>robot_speed:=` + (speed) + `</PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function writeVacuum(enable) {
    writeln(
`                   <Cmd ItemText="enable_vacuum:=` + (enable ? "true" : "false") + `">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_vacuum:=` + (enable ? "true" : "false") + `</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_vacuum:=` + (enable ? "true" : "false") + `</PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function writePause() {
    writeln(
`                   <Cmd ItemText="Pause">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Pause</CmdName>
                            <ShowName>Pause</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDPauseMode>0</Param0-IDPauseMode>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function writeDelay(delay) {
    writeln(
`                   <Cmd ItemText="Wait">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Wait</CmdName>
                            <ShowName>Wait</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDModeSet>0</Param0-IDModeSet>
                            <Param1-IDTimeMode>` + (delay) + `</Param1-IDTimeMode>
                            <Param2-IDExpressionMode>true</Param2-IDExpressionMode>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function writeMoveL(p1, blendRadius = 0.0, differPose = 2) {
    writeln(
`                    <Cmd ItemText="MoveL">
                        <Cmdinfo>
                            <CmdSetType>ECST_Move</CmdSetType>
                            <CmdName>MoveL</CmdName>
                            <ShowName>MoveL</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param1-IDVelocity>1.0</Param1-IDVelocity>
                            <Param2-IDAcc>1.0</Param2-IDAcc>
                            <Param3-IDJerk>1.0</Param3-IDJerk>
                            <Param4-IDRotVelocity>1.0</Param4-IDRotVelocity>
                            <Param5-IDRotAcc>1.0</Param5-IDRotAcc>
                            <Param6-IDRotJerk>1.0</Param6-IDRotJerk>
                        </Cmdinfo>
                        <ChildCmd>
                            <Cmd ItemText="PathPoint">
                                <Cmdinfo>
                                    <CmdSetType>ECST_Move</CmdSetType>
                                    <CmdName>PathPoint</CmdName>
                                    <ShowName>PathPoint</ShowName>
                                    <CmdId>` + (cmdID++) + `</CmdId>
                                    <SameTypeID>-1</SameTypeID>
                                    <Param0-IDTCP>0</Param0-IDTCP>
                                    <Param1-IDBase>0</Param1-IDBase>
                                    <Param2-IDUseCustom>false</Param2-IDUseCustom>
                                    <Param3-IDVelocity>1.0</Param3-IDVelocity>
                                    <Param4-IDAcc>1.0</Param4-IDAcc>
                                    <Param5-IDJerk>1.0</Param5-IDJerk>
                                    <Param6-IDRotVelocity>1.0</Param6-IDRotVelocity>
                                    <Param7-IDRotAcc>1.0</Param7-IDRotAcc>
                                    <Param8-IDRotJerk>1.0</Param8-IDRotJerk>
                                    <Param10-IDBlendSet>0</Param10-IDBlendSet>
                                    <Param11-IDRadius>` + (blendRadius) + `</Param11-IDRadius>
                                    <Param12-IDTimeScale>0</Param12-IDTimeScale>
                                    <Param15-IDModeset>0</Param15-IDModeset>
                                    <Param16-IDUseVel>1.05</Param16-IDUseVel>
                                    <Param17-IDTime>1</Param17-IDTime>
                                    <Param100-IDTabPointSet>''</Param100-IDTabPointSet>
                                    <Param101-IDPointset>1</Param101-IDPointset>
                                    <Param102-IDPointId>0</Param102-IDPointId>
                                    <Param103-IDPoint>-1</Param103-IDPoint>
                                    <Param104-IDX>` + (p1.x) + `</Param104-IDX>
                                    <Param105-IDY>` + (p1.y) + `</Param105-IDY>
                                    <Param106-IDZ>` + (p1.z) + `</Param106-IDZ>
                                    <Param109-IDRZ>` + ((p1.rz != null) ? p1.rz : 0) + `</Param109-IDRZ>
                                    <Param110-IDE1>0</Param110-IDE1>
                                    <Param111-IDE2>0</Param111-IDE2>
                                    <Param112-IDE3>0</Param112-IDE3>
                                    <Param113-IDE4>0</Param113-IDE4>
                                    <Param114-IDE5>0</Param114-IDE5>
                                    <Param115-IDE6>0</Param115-IDE6>
                                    <Param128-IDDifferPose>` + (differPose) + `</Param128-IDDifferPose>
                                </Cmdinfo>
                            </Cmd>
                            <Cmd ItemText="MoveL End"/>
                        </ChildCmd>
                    </Cmd>`
    );
}

function writeMoveC(p1, p2, blendRadius = 0.0, differPose = 2) {
    writeln(
 `                  <Cmd ItemText="MoveC">
                        <Cmdinfo>
                            <CmdSetType>ECST_Move</CmdSetType>
                            <CmdName>MoveC</CmdName>
                            <ShowName>MoveC</ShowName>
                            <CmdId>` + (cmdID++) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDTCP>0</Param0-IDTCP>
                            <Param1-IDBase>0</Param1-IDBase>
                            <Param3-IDVelocity>1</Param3-IDVelocity>
                            <Param4-IDAcc>1</Param4-IDAcc>
                            <Param5-IDJerk>1</Param5-IDJerk>
                            <Param6-IDRotVelocity>1</Param6-IDRotVelocity>
                            <Param7-IDRotAcc>1</Param7-IDRotAcc>
                            <Param8-IDRotJerk>1</Param8-IDRotJerk>
                            <Param9-IDBlendSet>0</Param9-IDBlendSet>
                            <Param10-IDRadius>` + (blendRadius) + `</Param10-IDRadius>
                            <Param11-IDTimeScale>0</Param11-IDTimeScale>
                            <Param13-IDDifferPose>` + (differPose) + `</Param13-IDDifferPose>
                            <Param14-IDModeset>0</Param14-IDModeset>
                            <Param15-IDUseVel>0.25</Param15-IDUseVel>
                            <Param16-IDTime>1</Param16-IDTime>
                            <Param18-TabPointSet1>''</Param18-TabPointSet1>
                            <Param19-IDPointset>1</Param19-IDPointset>
                            <Param20-IDPointId>0</Param20-IDPointId>
                            <Param21-IDPoint>-1</Param21-IDPoint>
                            <Param22-IDX>`+ (p1.x) +`</Param22-IDX>
                            <Param23-IDy>`+ (p1.y) +`</Param23-IDy>
                            <Param24-IDZ>`+ (p1.z) +`</Param24-IDZ>
                            <Param27-IDRZ>0</Param27-IDRZ>
                            <Param28-IDE1>0</Param28-IDE1>
                            <Param29-IDE2>0</Param29-IDE2>
                            <Param30-IDE3>0</Param30-IDE3>
                            <Param31-IDE4>0</Param31-IDE4>
                            <Param32-IDE5>0</Param32-IDE5>
                            <Param33-IDE6>0</Param33-IDE6>
                            <Param49-TabPointSet2>''</Param49-TabPointSet2>
                            <Param50-IDPointset2>1</Param50-IDPointset2>
                            <Param51-IDPointId2>0</Param51-IDPointId2>
                            <Param52-IDPoint2>-1</Param52-IDPoint2>
                            <Param53-IDX2>`+ (p2.x) +`</Param53-IDX2>
                            <Param54-IDY2>`+ (p2.y) +`</Param54-IDY2>
                            <Param55-IDZ2>`+ (p2.z) +`</Param55-IDZ2>
                            <Param58-IDRZ2>0</Param58-IDRZ2>
                            <Param59-IDE21>0</Param59-IDE21>
                            <Param60-IDE22>0</Param60-IDE22>
                            <Param61-IDE23>0</Param61-IDE23>
                            <Param62-IDE24>0</Param62-IDE24>
                            <Param63-IDE25>0</Param63-IDE25>
                            <Param64-IDE26>0</Param64-IDE26>
                        </Cmdinfo>
                    </Cmd>`
    );
}

function wrapDeg0_360(d) {
  d %= 360;
  if (d < 0) d += 360;
  return d;
}

function dist2D(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

// NOTE: Named unit2D to avoid Fusion's internal 'unit' symbol.
function unit2D(dx, dy) {
  const L = Math.hypot(dx, dy);
  if (L < EPS) return { x: 0, y: 0, ok: false };
  return { x: dx / L, y: dy / L, ok: true };
}

function signedArea2D(pts) {
  let A = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i], q = pts[(i + 1) % pts.length];
    A += (p.x * q.y - q.x * p.y);
  }
  return 0.5 * A;
}

/**
 * General angle: returns angle (deg) from arbitrary reference direction (refX, refY)
 * to vector (nx, ny), wrapped to [0,360).
 */
function angleFromReference(nx, ny, refX, refY) {
  const vLen = Math.hypot(nx, ny);
  const rLen = Math.hypot(refX, refY);
  if (vLen < EPS || rLen < EPS) return 0;
  nx /= vLen; ny /= vLen; refX /= rLen; refY /= rLen;

  const dot = nx * refX + ny * refY;
  const cross = refX * ny - refY * nx; // z-component of 2D cross
  const rad = Math.atan2(cross, dot);
  return wrapDeg0_360(rad * 180 / Math.PI);
}

/**
 * Shift angle 'a' (deg, in [0,360)) by ±360 to be the closest to 'prev',
 * then clamp to ±ANGLE_SPAN_LIMIT. Returns a signed angle (can be negative).
 */
function unwrapAngleTowards(prev, a) {
  // unwrap toward prev
  while (a - prev > 180) a -= 360;
  while (a - prev < -180) a += 360;

  // clamp to ±ANGLE_SPAN_LIMIT
  if (a > ANGLE_SPAN_LIMIT) a = ANGLE_SPAN_LIMIT;
  if (a < -ANGLE_SPAN_LIMIT) a = -ANGLE_SPAN_LIMIT;
  return a;
}

/**
 * Compute outward-facing yaw (deg) for each glue MoveL (mode === "feed").
 * Returns Map: programDraftIndex -> rzDeg (signed, continuous, limited to ±ANGLE_SPAN_LIMIT).
 */
function computeOutwardYawForGlueMoveL(programDraft) {
  // 1) Collect glue MoveL points (indices + XY)
  const idxs = [];
  const pts = [];
  for (let i = 0; i < programDraft.length; i++) {
    const it = programDraft[i];
    if (it.function === "writeMoveL" && it.arguments[1] === "feed") {
      const p = it.arguments[0];
      pts.push({ x: +p.x, y: +p.y });
      idxs.push(i);
    }
  }
  if (pts.length < 2) return new Map();

  // 2) Closed chain for normal computation
  const closed = pts.slice();
  if (dist2D(closed[0], closed[closed.length - 1]) > EPS) {
    closed.push({ x: closed[0].x, y: closed[0].y });
  }

  // 3) Winding: A>0 => CCW, A<0 => CW
  const A = signedArea2D(closed);
  const isCCW = A > 0;

  // 4) For each segment, choose outward normal (right if CCW, left if CW)
  const rzByIndex = new Map();
  for (let i = 0; i < pts.length; i++) {
    const p = closed[i], q = closed[i + 1];
    const t = unit2D(q.x - p.x, q.y - p.y);
    if (!t.ok) continue;

    const left = { x: -t.y, y:  t.x };
    const right= { x:  t.y, y: -t.x };
    const n = isCCW ? right : left;

    // Base angle from configured ZERO_DIR in [0,360)
    let rz = angleFromReference(n.x, n.y, ZERO_DIR.x, ZERO_DIR.y);

    // Apply controller sign and fixed tool offset, keep within [0,360)
    rz = wrapDeg0_360(ROT_SIGN * (rz + TOOL_OFFSET_DEG));

    // Assign to the vertex at the END of this segment
    const k = (i + 1) % pts.length;
    const draftIndex = idxs[k];
    rzByIndex.set(draftIndex, rz);
  }

  // 5) Build ordered list of glue MoveL indices for unwrapping continuity
  const glueIdxs = [];
  for (let i = 0; i < programDraft.length; i++) {
    const it = programDraft[i];
    if (it.function === "writeMoveL" && it.arguments[1] === "feed") {
      glueIdxs.push(i);
    }
  }
  if (glueIdxs.length === 0) return rzByIndex;

  // 6) Seed near 0° for the first angle (choose representation closest to 0)
  if (rzByIndex.has(glueIdxs[0])) {
    let a0 = rzByIndex.get(glueIdxs[0]);  // [0,360)
    if (a0 > 180) a0 -= 360;              // (-180,180]
    if (a0 > ANGLE_SPAN_LIMIT) a0 = ANGLE_SPAN_LIMIT;
    if (a0 < -ANGLE_SPAN_LIMIT) a0 = -ANGLE_SPAN_LIMIT;
    rzByIndex.set(glueIdxs[0], a0);

    // 7) Unwrap remaining angles toward previous, clamp to ±ANGLE_SPAN_LIMIT
    let prev = a0;
    for (let k = 1; k < glueIdxs.length; k++) {
      const idx = glueIdxs[k];
      if (!rzByIndex.has(idx)) continue;
      const base = rzByIndex.get(idx);    // [0,360)
      const unwrapped = unwrapAngleTowards(prev, base);
      rzByIndex.set(idx, unwrapped);
      prev = unwrapped;
    }
  }

  return rzByIndex;
}

// --- Apply approach/exit orientation: use the NEAREST (index-wise) rapid around EVERY glue run ---

// Find all contiguous glue runs (maximal consecutive writeMoveL with mode === "feed")
function findGlueRuns(programDraft) {
  const runs = [];
  let inRun = false, start = -1;

  for (let i = 0; i < programDraft.length; i++) {
    const it = programDraft[i];
    const isGlue = (it.function === "writeMoveL" && it.arguments[1] === "feed");

    if (isGlue && !inRun) { inRun = true; start = i; }
    const atEnd = (i === programDraft.length - 1);
    if ((inRun && !isGlue) || (inRun && isGlue && atEnd)) {
      const end = (inRun && isGlue && atEnd) ? i : (i - 1);
      runs.push({ start, end });
      inRun = false;
    }
  }
  return runs;
}

// --- Apply approach/exit orientation: tag the NEAREST N rapid moves around EVERY glue run ---
// Uses index-wise proximity (order), ignores non-move items, and does not cross into other glue runs.
// Depends on findGlueRuns(programDraft) defined above.

function applyApproachExitOrientation_AllRuns(programDraft, rzByIndex, rapidCountEachSide = 2) {
  const runs = findGlueRuns(programDraft);
  if (runs.length === 0) return;

  for (const { start: firstIdx, end: lastIdx } of runs) {
    const firstRZ = rzByIndex.get(firstIdx);
    const lastRZ  = rzByIndex.get(lastIdx);

    // ---------- BACKWARD: the nearest N rapids before first glue ----------
    if (firstRZ != null && rapidCountEachSide > 0) {
      let tagged = 0;
      for (let j = firstIdx - 1; j >= 0 && tagged < rapidCountEachSide; j--) {
        const it = programDraft[j];
        if (it.function === "writeMoveL") {
          const mode = it.arguments[1];
          if (mode === "rapid") {
            const p = it.arguments[0] || (programDraft[j].arguments[0] = {});
            p.rz = firstRZ;
            tagged++;
            continue;
          }
          if (mode === "feed") {
            // Do not cross into a previous glue run
            break;
          }
        }
        // ignore non-move items and keep scanning
      }
    }

    // ---------- FORWARD: the nearest N rapids after last glue ----------
    if (lastRZ != null && rapidCountEachSide > 0) {
      let tagged = 0;
      for (let j = lastIdx + 1; j < programDraft.length && tagged < rapidCountEachSide; j++) {
        const it = programDraft[j];
        if (it.function === "writeMoveL") {
          const mode = it.arguments[1];
          if (mode === "rapid") {
            const p = it.arguments[0] || (programDraft[j].arguments[0] = {});
            p.rz = lastRZ;
            tagged++;
            continue;
          }
          if (mode === "feed") {
            // Next glue run begins; stop
            break;
          }
        }
        // ignore non-move items and keep scanning
      }
    }
  }
}

/**
 * Insert a zero-length "pre-rotate" point at the start of long straight segments.
 * For each glue segment k -> k+1 in every glue run:
 *  - If segment length >= minLen, insert an extra writeMoveL (feed) right AFTER point k
 *    at the SAME XYZ with RZ equal to the segment’s outward RZ (stored at index k+1).
 * This causes a brief stop+rotate at the corner, then the long straight is traversed
 * already aligned. Works nicely with blend radius (we're not adding tiny moves).
 *
 * Depends on: findGlueRuns(), xyzFormat, rzByIndex (computed at glue endpoints).
 */
function applyCornerPreRotateLongSegments(programDraft, rzByIndex, minLen, xyzFormat) {
  const runs = findGlueRuns(programDraft);
  if (runs.length === 0) return programDraft;

  const newDraft = [];
  const EPS_LEN = 1e-9;

  function segLen(A, B, cachedLen) {
    if (typeof cachedLen === "number") return +cachedLen;
    const dx = (+B.x) - (+A.x);
    const dy = (+B.y) - (+A.y);
    return Math.hypot(dx, dy);
  }

  for (let i = 0; i < programDraft.length; i++) {
    const it = programDraft[i];

    // If not a glue move, just pass through
    if (!(it.function === "writeMoveL" && it.arguments[1] === "feed")) {
      newDraft.push(it);
      continue;
    }

    // Find the glue run containing i
    const run = (() => {
      for (const r of runs) if (i >= r.start && i <= r.end) return r;
      return null;
    })();

    if (!run) { newDraft.push(it); continue; }

    // Process the whole run once; we will advance i = run.end afterwards
    for (let k = run.start; k <= run.end; k++) {
      const Aitem = programDraft[k];
      const A = Aitem.arguments[0];

      // Push the original point A
      newDraft.push(Aitem);

      // If there is a next glue point, evaluate segment A->B
      if (k < run.end) {
        const Bitem = programDraft[k + 1];
        const B = Bitem.arguments[0];
        const cachedLen = Aitem.arguments[2]; // your onLinear() already stores length here
        const L = segLen(A, B, cachedLen);

        if (L >= Math.max(0, +minLen || 0) + EPS_LEN) {
          // If we have a segment outward RZ computed at B index (k+1), use it
          if (rzByIndex.has(k + 1)) {
            const segRZ = rzByIndex.get(k + 1);

            // Insert zero-length pre-rotate point A* immediately AFTER A
            const Astar = {
              x: +xyzFormat.format(+A.x),
              y: +xyzFormat.format(+A.y),
              z: +xyzFormat.format(+A.z),
              rz: segRZ
            };

            newDraft.push({
              function: "writeMoveL",
              arguments: [ Astar, "feed", 0 ] // length=0 ⇒ explicit stop+rotate
            });
          }
        }
      }
    }

    // Skip to end of this run in outer loop
    i = run.end;
  }

  return newDraft;
}

// Write rzByIndex directly onto the existing glue points (point objects),
// so later insertions won't break mapping by index.
function assignGlueRZToPoints(programDraft, rzByIndex) {
  for (let i = 0; i < programDraft.length; i++) {
    const it = programDraft[i];
    if (it.function === "writeMoveL" && it.arguments[1] === "feed" && rzByIndex.has(i)) {
      const p = it.arguments[0] || (it.arguments[0] = {});
      p.rz = rzByIndex.get(i);
    }
  }
}
