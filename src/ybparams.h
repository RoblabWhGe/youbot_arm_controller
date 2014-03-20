#ifndef YBPARAMS_H
#define YBPARAMS_H

#include <cmath>
#include <string>

/** Unteres Limit der Standard-Winkelwerte. */
const double BOTTOM_LIMIT_SD[5] = {-2.949606, // -169 Grad
                                   -1.134464, // -65 Grad
                                   -2.635447, // -151 Grad
                                   -1.788962, // -102.5 Grad
                                   -2.923426};// -167.5 Grad

/** Oberes Limit der Standard-Winkelwerte. */
const double TOP_LIMIT_SD[5] = {2.949606, // 169 Grad
                                1.570796, // 90 Grad
                                2.548181, // 146 Grad
                                1.788962, // 102.5 Grad
                                2.923426};// 167.5 Grad

/** Unteres Limit der Youbot-Winkelwerte. */
const double BOTTOM_LIMIT_YB[5] = {0.0100692,
                                   0.0100692,
                                   -5.02655,
                                   0.0221239,
                                   0.110619};

/** Oberes Limit der Youbot-Winkelwerte. */
const double TOP_LIMIT_YB[5] = {5.84014,
                                2.61799,
                                -0.015708,
                                3.4292,
                                5.64159};
/** Unteres und oberes Limit des Arm-Grippers. */
const double GRIPPER_LIMIT[2] = {0,
                                 0.023};

/** Laengen des Youbot-Arms. */
const double LENGTHS[5] = {0.075,
                           0.033,
                           0.155,
                           0.135,
                           0.218};

/** DH-Parameter: Drehung um Z-Achse. */
const double DH_THETA[5] = {0,
                            -M_PI_2,
                            0,
                            M_PI_2,
                            M_PI_2};

/** DH-Parameter: Verschiebung entlang Z-Achse. */
const double DH_D[5] = {LENGTHS[0],
                        0,
                        0,
                        0,
                        LENGTHS[4]};

/** DH-Parameter: Verschiebung entlang X-Achse. */
const double DH_R[5] = {LENGTHS[1],
                        LENGTHS[2],
                        LENGTHS[3],
                        0,
                        0};

/** DH-Parameter: Drehung um X-Achse. */
const double DH_ALPHA[5] = {-M_PI_2,
                            0,
                            0,
                            M_PI_2,
                            0};

/** Arm-Gelenk-Namen. */
const std::string ARM_JOINT_NAMES[7] = {"youbot_arm_base",
                                        "youbot_arm_link_0",
                                        "youbot_arm_link_1",
                                        "youbot_arm_link_2",
                                        "youbot_arm_link_3",
                                        "youbot_arm_link_4",
                                        "youbot_tcp"};

#endif // YBPARAMS_H
