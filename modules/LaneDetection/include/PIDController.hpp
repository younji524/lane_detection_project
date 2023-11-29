/**
 * @file PIDController.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the PIDController class for applying pid control to the system in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__PIDCONTROLLER_HPP
#define LANE_DETECTION__PIDCONTROLLER_HPP

// User defined header
#include "Common.hpp"

namespace XyCar
{
/**
 * @brief PIDController Class.
 * @details This class is used to apply PID control.
 */
class PIDController
{
public:
  using Ptr = PIDController *;  ///< Pointer type of class.

  /**
   * @brief Construct a new PID Object.
   * @details This function constructs a new PID Object.
   * @param[in] p_gain Proportional control gain.
   * @param[in] i_gain Integral control gain.
   * @param[in] d_gain Differential control gain.
   */
  PIDController(PREC p_gain, PREC i_gain, PREC d_gain);

  /**
   * @brief Compute with the PID Control and return control error.
   * @details This function computes with the PID Control and return control error.
   * @param[in] error Error between the estimated x coordinates and half of the image.
   * @return XyCar::PREC Control error (Here, PREC refers to double)
   */
  PREC compute_angle(int32_t error);

private:
  PREC proportional_gain_;  ///< The proportional gain for pid control.
  PREC integral_gain_;  ///< The integral gain for pid control.
  PREC differential_gain_;  ///< The differential gain for pid control.

  PREC proportional_error_ = 0.0;  ///< The proportional error for pid control.
  PREC integral_error_ = 0.0;  ///< The integral error for pid control.
  PREC differential_error_ = 0.0;  ///< The differential error for pid control.
};
} // namespace XyCar

#endif // LANE_DETECTION__PIDCONTROLLER_HPP
