description = "Robot Phoenix SCARA Postprocessor";
vendor = "BNRobotics";
legal = "MIT License";
extension = "xml";

setCodePage("ascii");

capabilities = CAPABILITY_MILLING | CAPABILITY_JET;
var cmdID = 0;
var epsilon = 20.0;
var BlendRadius = 10.0;

var glueEnabled = false;
var maxSpeed = 0.5;

var lastPosition = { x: 0, y: 0, z: 0 };
var lastSpeed = 0.1; // Initialize with homing speed.

var xyzFormat = createFormat({decimals: 2, trim: true});
var feedFormat = createFormat({decimals: 2, trim: true});

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
    // blendingRadius: {
    //     title: "Blending Radius (mm)",
    //     // description: "The robot speed fraction for fast movement without glue application.",
    //     group: "2 - Advanced",
    //     type: "number",
    //     value: 10,
    //     // range: [0, maxVelocityFraction],
    //     scope: "post",
    // },
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
            <CmdId>0</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="Call Initialize">
                <Cmdinfo>
                    <CmdSetType>ECST_Logic</CmdSetType>
                    <CmdName>Call Subprogram</CmdName>
                    <ShowName>Call Initialize</ShowName>
                    <CmdId>1</CmdId>
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
            <CmdId>2</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetRobot">
                <Cmdinfo>
                    <CmdSetType>ECST_Segment</CmdSetType>
                    <CmdName>SetRobot</CmdName>
                    <ShowName>SetRobot</ShowName>
                    <CmdId>3</CmdId>
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
                            <CmdId>4</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_glue:=false </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>
                    <Cmd ItemText="enable_vacuum:=false">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_vacuum:=false</ShowName>
                            <CmdId>5</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_vacuum:=false </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>
                    <Cmd ItemText="SetHomingSpeed">
                        <Cmdinfo>
                            <CmdSetType>ECST_Set</CmdSetType>
                            <CmdName>SetFixedSpeed</CmdName>
                            <ShowName>SetHomingSpeed</ShowName>
                            <CmdId>7</CmdId>
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
                            <CmdId>8</CmdId>
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
            <CmdId>10</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="Call Initialize">
                <Cmdinfo>
                    <CmdSetType>ECST_Logic</CmdSetType>
                    <CmdName>Call Subprogram</CmdName>
                    <ShowName>Call Initialize</ShowName>
                    <CmdId>11</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDSubprogram>Initialize Subprogram</Param0-IDSubprogram>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="SetRobot">
                <Cmdinfo>
                    <CmdSetType>ECST_Segment</CmdSetType>
                    <CmdName>SetRobot</CmdName>
                    <ShowName>SetRobot</ShowName>
                    <CmdId>12</CmdId>
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
                            <CmdId>13</CmdId>
                            <SameTypeID>0</SameTypeID>
                            <PmExp0-IDTimes>true </PmExp0-IDTimes>
                        </Cmdinfo>
                        <ChildCmd>`
  );
  cmdID = 14;
}

function onClose() {
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
            <CmdId>` + (cmdID + 1) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetGlueOutput">
                <Cmdinfo>
                    <CmdSetType>ECST_IO</CmdSetType>
                    <CmdName>SetOutput</CmdName>
                    <ShowName>SetGlueOutput</ShowName>
                    <CmdId>` + (cmdID + 2) + `</CmdId>
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
            <CmdId>` + (cmdID + 3) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetVacuumTableOutput">
                <Cmdinfo>
                    <CmdSetType>ECST_IO</CmdSetType>
                    <CmdName>SetOutput</CmdName>
                    <ShowName>SetVacuumTableOutput</ShowName>
                    <CmdId>` + (cmdID + 4) + `</CmdId>
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
            <CmdId>` + (cmdID + 5) + `</CmdId>
            <SameTypeID>-1</SameTypeID>
        </Cmdinfo>
        <ChildCmd>
            <Cmd ItemText="SetOperationSpeed">
                <Cmdinfo>
                    <CmdSetType>ECST_Set</CmdSetType>
                    <CmdName>SetFixedSpeed</CmdName>
                    <ShowName>SetOperationSpeed</ShowName>
                    <CmdId>` + (cmdID + 6) + `</CmdId>
                    <SameTypeID>-1</SameTypeID>
                    <Param0-IDFixedSpeed>true</Param0-IDFixedSpeed>
                    <PmVar1-IDSpeedRatio>1</PmVar1-IDSpeedRatio>
                </Cmdinfo>
            </Cmd>
            <Cmd ItemText="Thread End"/>
        </ChildCmd>
    </Cmd>
</Cmds>`);
}

function onParameter(name, value) {
    var zOffset = getProperty("zTableOffset");
    var rapidMotionSpeed = getProperty("rapidMotionSpeed");

    switch (name) {
        case "action":
            var sText1 = String(value).toUpperCase();
            var sText2 = new Array();
            sText2 = sText1.split(" ");

        switch (sText2[0]) {
            case "VACUUM":
            if (sText2[1] == "ON") {
                    writeln(
`                   <Cmd ItemText="enable_vacuum:=true">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_vacuum:=true</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_vacuum:=true </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>`);
                cmdID += 1;

          } else if (sText2[1] == "OFF") {
                writeln(
`                   <Cmd ItemText="enable_vacuum:=false">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_vacuum:=false</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_vacuum:=false </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>`);
                cmdID += 1;
                cmdID += 1;
            }
            
            break;

        case "PAUSE":
            writeln(
`                   <Cmd ItemText="Pause">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Pause</CmdName>
                            <ShowName>Pause</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDPauseMode>0</Param0-IDPauseMode>
                        </Cmdinfo>
                    </Cmd>`);
            cmdID += 1;

            break;

        case "DELAY":
            writeln(
`                   <Cmd ItemText="Wait">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Wait</CmdName>
                            <ShowName>Wait</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <Param0-IDModeSet>0</Param0-IDModeSet>
                            <Param1-IDTimeMode>` + (parseFloat(sText2[1])) + `</Param1-IDTimeMode>
                            <Param2-IDExpressionMode>true</Param2-IDExpressionMode>
                        </Cmdinfo>
                    </Cmd>`);
            cmdID += 1;

            break;

        case "MOVE":
            x = parseFloat(sText2[1]);
            y = parseFloat(sText2[2]);
            z = parseFloat(sText2[3]) + zOffset;

            if (Math.abs(lastSpeed - rapidMotionSpeed) > 0.01) {    
                writeln(
`                   <Cmd ItemText="robot_speed:=` + rapidMotionSpeed + `">
                    <Cmdinfo>
                        <CmdSetType>ECST_Logic</CmdSetType>
                        <CmdName>Expression</CmdName>
                        <ShowName>robot_speed:=` + rapidMotionSpeed + `</ShowName>
                        <CmdId>` + (cmdID) + `</CmdId>
                        <SameTypeID>-1</SameTypeID>
                        <PmExp0-IDExpression>robot_speed:=` + rapidMotionSpeed + `</PmExp0-IDExpression>
                    </Cmdinfo>
                </Cmd>`);
                lastSpeed = rapidMotionSpeed;
                cmdID += 1;
            }

            writeln(
`                    <Cmd ItemText="MoveL">
                        <Cmdinfo>
                            <CmdSetType>ECST_Move</CmdSetType>
                            <CmdName>MoveL</CmdName>
                            <ShowName>MoveL</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
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
                                    <CmdId>` + (cmdID + 1) + `</CmdId>
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
                                    <Param11-IDRadius>0</Param11-IDRadius>
                                    <Param12-IDTimeScale>0</Param12-IDTimeScale>
                                    <Param15-IDModeset>0</Param15-IDModeset>
                                    <Param16-IDUseVel>1.05</Param16-IDUseVel>
                                    <Param17-IDTime>1</Param17-IDTime>
                                    <Param100-IDTabPointSet>''</Param100-IDTabPointSet>
                                    <Param101-IDPointset>1</Param101-IDPointset>
                                    <Param102-IDPointId>0</Param102-IDPointId>
                                    <Param103-IDPoint>-1</Param103-IDPoint>
                                    <Param104-IDX>` + (x) + `</Param104-IDX>
                                    <Param105-IDY>` + (y) + `</Param105-IDY>
                                    <Param106-IDZ>` + (z) + `</Param106-IDZ>
                                    <Param109-IDRZ>0</Param109-IDRZ>
                                    <Param110-IDE1>0</Param110-IDE1>
                                    <Param111-IDE2>0</Param111-IDE2>
                                    <Param112-IDE3>0</Param112-IDE3>
                                    <Param113-IDE4>0</Param113-IDE4>
                                    <Param114-IDE5>0</Param114-IDE5>
                                    <Param115-IDE6>0</Param115-IDE6>
                                    <Param128-IDDifferPose>2</Param128-IDDifferPose>
                                </Cmdinfo>
                            </Cmd>
                            <Cmd ItemText="MoveL End"/>
                        </ChildCmd>
                    </Cmd>`);
            cmdID += 2;

            break;
        }
    }
}

function onRapid(__x, __y, __z) {
    var zOffset = getProperty("zTableOffset");
    var rapidMotionSpeed = getProperty("rapidMotionSpeed");
    var x = parseFloat(xyzFormat.format(__x));
    var y = parseFloat(xyzFormat.format(__y));
    var z = parseFloat(xyzFormat.format(__z)) + zOffset;

    if (glueEnabled == true) {
        writeln(
`                   <Cmd ItemText="enable_glue:=false">
                    <Cmdinfo>
                        <CmdSetType>ECST_Logic</CmdSetType>
                        <CmdName>Expression</CmdName>
                        <ShowName>enable_glue:=false</ShowName>
                        <CmdId>` + (cmdID) + `</CmdId>
                        <SameTypeID>-1</SameTypeID>
                        <PmExp0-IDExpression>enable_glue:=false</PmExp0-IDExpression>
                    </Cmdinfo>
                </Cmd>`);
        glueEnabled = false;
        cmdID += 1;
    }

    if (Math.abs(lastSpeed - rapidMotionSpeed) > 0.01) {    
        writeln(
`                   <Cmd ItemText="robot_speed:=` + rapidMotionSpeed + `">
                    <Cmdinfo>
                        <CmdSetType>ECST_Logic</CmdSetType>
                        <CmdName>Expression</CmdName>
                        <ShowName>robot_speed:=` + rapidMotionSpeed + `</ShowName>
                        <CmdId>` + (cmdID) + `</CmdId>
                        <SameTypeID>-1</SameTypeID>
                        <PmExp0-IDExpression>robot_speed:=` + rapidMotionSpeed + `</PmExp0-IDExpression>
                    </Cmdinfo>
                </Cmd>`);
        lastSpeed = rapidMotionSpeed;
        cmdID += 1;
    }

    writeln(
`                    <Cmd ItemText="MoveL">
                          <Cmdinfo>
                              <CmdSetType>ECST_Move</CmdSetType>
                              <CmdName>MoveL</CmdName>
                              <ShowName>MoveL</ShowName>
                              <CmdId>` + (cmdID) + `</CmdId>
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
                                      <CmdId>` + (cmdID + 1) + `</CmdId>
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
                                      <Param11-IDRadius>0</Param11-IDRadius>
                                      <Param12-IDTimeScale>0</Param12-IDTimeScale>
                                      <Param15-IDModeset>0</Param15-IDModeset>
                                      <Param16-IDUseVel>1.05</Param16-IDUseVel>
                                      <Param17-IDTime>1</Param17-IDTime>
                                      <Param100-IDTabPointSet>''</Param100-IDTabPointSet>
                                      <Param101-IDPointset>1</Param101-IDPointset>
                                      <Param102-IDPointId>0</Param102-IDPointId>
                                      <Param103-IDPoint>-1</Param103-IDPoint>
                                      <Param104-IDX>` + (x) + `</Param104-IDX>
                                      <Param105-IDY>` + (y) + `</Param105-IDY>
                                      <Param106-IDZ>` + (z) + `</Param106-IDZ>
                                      <Param109-IDRZ>0</Param109-IDRZ>
                                      <Param110-IDE1>0</Param110-IDE1>
                                      <Param111-IDE2>0</Param111-IDE2>
                                      <Param112-IDE3>0</Param112-IDE3>
                                      <Param113-IDE4>0</Param113-IDE4>
                                      <Param114-IDE5>0</Param114-IDE5>
                                      <Param115-IDE6>0</Param115-IDE6>
                                      <Param128-IDDifferPose>2</Param128-IDDifferPose>
                                  </Cmdinfo>
                              </Cmd>
                              <Cmd ItemText="MoveL End"/>
                          </ChildCmd>
                      </Cmd>`);
    cmdID += 2;
}

function onLinear(__x, __y, __z, __feed) {
    var zOffset = getProperty("zTableOffset");
    var x = parseFloat(xyzFormat.format(__x));
    var y = parseFloat(xyzFormat.format(__y));
    var z = parseFloat(xyzFormat.format(__z)) + zOffset;
    var feed = parseFloat(feedFormat.format(__feed));

    if (Math.abs(x - lastPosition.x) < epsilon && Math.abs(y - lastPosition.y) < epsilon && Math.abs(z - lastPosition.z) < epsilon) {
      return;
    }

    lastPosition = { x: x, y: y, z: z };

    if (glueEnabled == false) {
        writeln(
`                   <Cmd ItemText="enable_glue:=true">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Expression</CmdName>
                            <ShowName>enable_glue:=true</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
                            <SameTypeID>-1</SameTypeID>
                            <PmExp0-IDExpression>enable_glue:=true </PmExp0-IDExpression>
                        </Cmdinfo>
                    </Cmd>`);
        glueEnabled = true;
        cmdID += 1;
    }

    if (Math.abs(lastSpeed - feed) > 0.01) {    
        writeln(
`                   <Cmd ItemText="robot_speed:=` + feed + `">
                    <Cmdinfo>
                        <CmdSetType>ECST_Logic</CmdSetType>
                        <CmdName>Expression</CmdName>
                        <ShowName>robot_speed:=` + feed + `</ShowName>
                        <CmdId>` + (cmdID) + `</CmdId>
                        <SameTypeID>-1</SameTypeID>
                        <PmExp0-IDExpression>robot_speed:=` + feed + `</PmExp0-IDExpression>
                    </Cmdinfo>
                </Cmd>`);
        lastSpeed = feed;
        cmdID += 1;
    }

    writeln(
`                    <Cmd ItemText="MoveL">
                        <Cmdinfo>
                            <CmdSetType>ECST_Move</CmdSetType>
                            <CmdName>MoveL</CmdName>
                            <ShowName>MoveL</ShowName>
                            <CmdId>` + (cmdID) + `</CmdId>
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
                                    <CmdId>` + (cmdID + 1) + `</CmdId>
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
                                    <Param11-IDRadius>` + (BlendRadius) + `</Param11-IDRadius>
                                    <Param12-IDTimeScale>0</Param12-IDTimeScale>
                                    <Param15-IDModeset>0</Param15-IDModeset>
                                    <Param16-IDUseVel>1.05</Param16-IDUseVel>
                                    <Param17-IDTime>1</Param17-IDTime>
                                    <Param100-IDTabPointSet>''</Param100-IDTabPointSet>
                                    <Param101-IDPointset>1</Param101-IDPointset>
                                    <Param102-IDPointId>0</Param102-IDPointId>
                                    <Param103-IDPoint>-1</Param103-IDPoint>
                                    <Param104-IDX>` + (x) + `</Param104-IDX>
                                    <Param105-IDY>` + (y) + `</Param105-IDY>
                                    <Param106-IDZ>` + (z) + `</Param106-IDZ>
                                    <Param109-IDRZ>0</Param109-IDRZ>
                                    <Param110-IDE1>0</Param110-IDE1>
                                    <Param111-IDE2>0</Param111-IDE2>
                                    <Param112-IDE3>0</Param112-IDE3>
                                    <Param113-IDE4>0</Param113-IDE4>
                                    <Param114-IDE5>0</Param114-IDE5>
                                    <Param115-IDE6>0</Param115-IDE6>
                                    <Param128-IDDifferPose>2</Param128-IDDifferPose>
                                </Cmdinfo>
                            </Cmd>
                            <Cmd ItemText="MoveL End"/>
                        </ChildCmd>
                    </Cmd>`);
  cmdID += 2;
}

function onCircular(__clockwise, ___cx, __cy, __cz, __x, __y, __z) {
}

function onSection() {
    var currentSection = getSection(getCurrentSectionId());
    if (currentSection.hasParameter("operation-comment")) {
        var comment = currentSection.getParameter("operation-comment");
        if (comment) {
            writeln(
`                    <Cmd ItemText="`+ `; Start: ` + comment +`">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Label</CmdName>
                            <ShowName>`+ `; Start: ` + comment +`</ShowName>
                            <CmdId>`+ cmdID +`</CmdId>
                            <SameTypeID>-1</SameTypeID>
                        </Cmdinfo>
                    </Cmd>`);
            cmdID += 1;
        }  
    }     
}

function onSectionEnd() {
    var currentSection = getSection(getCurrentSectionId());
    if (currentSection.hasParameter("operation-comment")) {
        var comment = currentSection.getParameter("operation-comment");
        if (comment) {
            writeln(
`                    <Cmd ItemText="`+ `; End: ` + comment +`">
                        <Cmdinfo>
                            <CmdSetType>ECST_Logic</CmdSetType>
                            <CmdName>Label</CmdName>
                            <ShowName>`+ `; End: ` + comment +`</ShowName>
                            <CmdId>`+ cmdID +`</CmdId>
                            <SameTypeID>-1</SameTypeID>
                        </Cmdinfo>
                    </Cmd>`);
            cmdID += 1;
        }  
    }     
}