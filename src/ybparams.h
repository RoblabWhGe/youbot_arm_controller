#ifndef YBPARAMS_H
#define YBPARAMS_H

#include <cmath>
#include <string>

/** Bottom limits of the angles defined in the datasheet. */
const double BOTTOM_LIMIT_SD[5] = {-2.949606, // -169 Degree
                                   -1.134464, // -65 Degree
                                   -2.635447, // -151 Degree
                                   -1.788962, // -102.5 Degree
                                   -2.923426};// -167.5 Degree

/** Top limits of the angles defined in the datasheet. */
const double TOP_LIMIT_SD[5] = {2.949606, // 169 Degree
                                1.570796, // 90 Degree
                                2.548181, // 146 Degree
                                1.788962, // 102.5 Degree
                                2.923426};// 167.5 Degree

/** Bottom limits of the angles defined by the motor controllers */
const double BOTTOM_LIMIT_YB[5] = {0.0100692,
                                   0.0100692,
                                   -5.02655,
                                   0.0221239,
                                   0.110619};

/** Top limits of the angles defined by the motor controllers */
const double TOP_LIMIT_YB[5] = {5.84014,
                                2.61799,
                                -0.015708,
                                3.4292,
                                5.64159};

/** Limits of the gripper space */
const double GRIPPER_LIMIT[2] = {0,
                                 0.023};

/** DH-Parameter: Theta */
const double DH_THETA[5] = {0,
                            -M_PI_2,
                            0,
                            M_PI_2,
                            0};

/** DH-Parameter: D */
const double DH_D[5] = {0.147,
                        0,
                        0,
                        0,
                        0.171};

/** DH-Parameter: R */
const double DH_R[5] = {0.033,
                        0.155,
                        0.135,
                        0,
                        0};

/** DH-Parameter: Alpha */
const double DH_ALPHA[5] = {-M_PI_2,
                            0,
                            0,
                            M_PI_2,
                            0};

#endif // YBPARAMS_H
