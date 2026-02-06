// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.can;

/** Add your docs here. */
public final class CanConstants {

  /** Stores our canbus names. */
  public static enum CanBusName {
    RIO("rio"),
    CTRE("ctre");

    public final String name;

    private CanBusName(String busName) {
      name = busName;
    }
  }

  /**
   * This class will help quickly find references to all CanIds (besides from the generated CTRE
   * swerve template)
   */
  public static enum CanId {
    // 0-19, reserved for TunerConstants

    // 20s Climber
    ID_20(20),
    ID_21(21),
    ID_22(22),
    ID_23(23),
    ID_24(24),
    ID_25(25),
    ID_26(26),
    ID_27(27),
    ID_28(28),
    ID_29(29),

    // 30s Intake
    ID_30(30),
    ID_31(31),
    ID_32(32),
    ID_33(33),
    ID_34(34),
    ID_35(35),
    ID_36(36),
    ID_37(37),
    ID_38(38),
    ID_39(39),

    // 40s Launcher
    ID_40(40),
    ID_41(41),
    ID_42(42),
    ID_43(43),
    ID_44(44),
    ID_45(45),
    ID_46(46),
    ID_47(47),
    ID_48(48),
    ID_49(49),

    // 50s
    ID_50(50),
    ID_51(51),
    ID_52(52),
    ID_53(53),
    ID_54(54),
    ID_55(55),
    ID_56(56),
    ID_57(57),
    ID_58(58),
    ID_59(59),

    // 60s
    ID_60(60),
    ID_61(61),
    ID_62(62);

    public final int id;

    private CanId(int canIdNumber) {
      id = canIdNumber;
    }
  }
}
