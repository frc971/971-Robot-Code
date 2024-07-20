#include <symengine/add.h>
#include <symengine/matrix.h>
#include <symengine/number.h>
#include <symengine/printers.h>
#include <symengine/real_double.h>
#include <symengine/simplify.h>
#include <symengine/solve.h>
#include <symengine/symbol.h>

#include <array>
#include <cmath>
#include <numbers>
#include <utility>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/substitute.h"

#include "aos/init.h"
#include "aos/util/file.h"
#include "frc971/control_loops/swerve/motors.h"

ABSL_FLAG(std::string, output_base, "",
          "Path to strip off the front of the output paths.");
ABSL_FLAG(std::string, cc_output_path, "",
          "Path to write generated cc code to");
ABSL_FLAG(std::string, h_output_path, "",
          "Path to write generated header code to");
ABSL_FLAG(std::string, py_output_path, "",
          "Path to write generated py code to");
ABSL_FLAG(std::string, casadi_py_output_path, "",
          "Path to write casadi generated py code to");
ABSL_FLAG(double, caster, 0.01, "Caster in meters for the module.");

ABSL_FLAG(bool, symbolic, false, "If true, write everything out symbolically.");

using SymEngine::abs;
using SymEngine::add;
using SymEngine::atan2;
using SymEngine::Basic;
using SymEngine::ccode;
using SymEngine::cos;
using SymEngine::DenseMatrix;
using SymEngine::div;
using SymEngine::Inf;
using SymEngine::integer;
using SymEngine::map_basic_basic;
using SymEngine::minus_one;
using SymEngine::neg;
using SymEngine::NegInf;
using SymEngine::pow;
using SymEngine::RCP;
using SymEngine::real_double;
using SymEngine::RealDouble;
using SymEngine::Set;
using SymEngine::sign;
using SymEngine::simplify;
using SymEngine::sin;
using SymEngine::solve;
using SymEngine::symbol;
using SymEngine::Symbol;

namespace frc971::control_loops::swerve {

// State per module.
struct Module {
  RCP<const Symbol> Is;

  RCP<const Symbol> Id;

  RCP<const Symbol> thetas;
  RCP<const Symbol> omegas;
  RCP<const Symbol> alphas;
  RCP<const Basic> alphas_eqn;

  RCP<const Symbol> thetad;
  RCP<const Symbol> omegad;
  RCP<const Symbol> alphad;
  RCP<const Basic> alphad_eqn;

  DenseMatrix contact_patch_velocity;
  DenseMatrix wheel_ground_velocity;
  DenseMatrix wheel_slip_velocity;
  RCP<const Basic> slip_angle;
  RCP<const Basic> slip_ratio;

  RCP<const Basic> Ms;
  RCP<const Basic> Fwx;
  RCP<const Basic> Fwy;
  DenseMatrix F;
  DenseMatrix mounting_location;

  // Acceleration contribution from this module.
  DenseMatrix accel;
  RCP<const Basic> angular_accel;
};

class SwerveSimulation {
 public:
  SwerveSimulation() : drive_motor_(KrakenFOC()), steer_motor_(KrakenFOC()) {
    auto fx = symbol("fx");
    auto fy = symbol("fy");
    auto moment = symbol("moment");

    if (absl::GetFlag(FLAGS_symbolic)) {
      Cx_ = symbol("Cx");
      Cy_ = symbol("Cy");

      rw_ = symbol("rw");

      m_ = symbol("m");
      J_ = symbol("J");

      Gd1_ = symbol("Gd1");
      rs_ = symbol("rs");
      rp_ = symbol("rp");
      Gd2_ = symbol("Gd2");

      rb1_ = symbol("rb1");
      rb2_ = symbol("rb2");

      Gd2_ = symbol("Gd3");
      Gd_ = symbol("Gd");

      Js_ = symbol("Js");

      Gs_ = symbol("Gs");
      wb_ = symbol("wb");

      Jdm_ = symbol("Jdm");
      Jsm_ = symbol("Jsm");
      Kts_ = symbol("Kts");
      Ktd_ = symbol("Ktd");

      robot_width_ = symbol("robot_width");

      caster_ = symbol("caster");
      contact_patch_length_ = symbol("Lcp");
    } else {
      Cx_ = real_double(25.0 * 9.8 / 4.0 / 0.05);
      Cy_ = real_double(5 * 9.8 / 0.05 / 4.0);

      rw_ = real_double(2 * 0.0254);

      m_ = real_double(25.0);  // base is 20 kg without battery
      J_ = real_double(6.0);

      Gd1_ = real_double(12.0 / 42.0);
      rs_ = real_double(28.0 / 20.0 / 2.0);
      rp_ = real_double(18.0 / 20.0 / 2.0);
      Gd2_ = div(rs_, rp_);

      // 15 / 45 bevel ratio, calculated using python script ported over to
      // GetBevelPitchRadius(double
      // TODO(Justin): Use the function instead of computed constantss
      rb1_ = real_double(0.3805473);
      rb2_ = real_double(1.14164);

      Gd3_ = div(rb1_, rb2_);
      Gd_ = mul(mul(Gd1_, Gd2_), Gd3_);

      Js_ = real_double(0.001);

      Gs_ = real_double(35.0 / 468.0);
      wb_ = real_double(0.725);

      Jdm_ = real_double(drive_motor_.motor_inertia);
      Jsm_ = real_double(steer_motor_.motor_inertia);
      Kts_ = real_double(steer_motor_.Kt);
      Ktd_ = real_double(drive_motor_.Kt);

      robot_width_ = real_double(24.75 * 0.0254);

      caster_ = real_double(absl::GetFlag(FLAGS_caster));
      contact_patch_length_ = real_double(0.02);
    }

    x_ = symbol("x");
    y_ = symbol("y");
    theta_ = symbol("theta");

    vx_ = symbol("vx");
    vy_ = symbol("vy");
    omega_ = symbol("omega");

    ax_ = symbol("ax");
    ay_ = symbol("ay");
    atheta_ = symbol("atheta");

    // Now, compute the accelerations due to the disturbance forces.
    DenseMatrix external_accel = DenseMatrix(2, 1, {div(fx, m_), div(fy, m_)});

    // And compute the physics contributions from each module.
    modules_[0] = ModulePhysics(
        0, DenseMatrix(
               2, 1,
               {div(robot_width_, integer(2)), div(robot_width_, integer(2))}));
    modules_[1] =
        ModulePhysics(1, DenseMatrix(2, 1,
                                     {div(robot_width_, integer(-2)),
                                      div(robot_width_, integer(2))}));
    modules_[2] =
        ModulePhysics(2, DenseMatrix(2, 1,
                                     {div(robot_width_, integer(-2)),
                                      div(robot_width_, integer(-2))}));
    modules_[3] =
        ModulePhysics(3, DenseMatrix(2, 1,
                                     {div(robot_width_, integer(2)),
                                      div(robot_width_, integer(-2))}));

    // And convert them into the overall robot contribution.
    DenseMatrix temp0 = DenseMatrix(2, 1);
    DenseMatrix temp1 = DenseMatrix(2, 1);
    DenseMatrix temp2 = DenseMatrix(2, 1);
    accel_ = DenseMatrix(2, 1);

    add_dense_dense(modules_[0].accel, external_accel, temp0);
    add_dense_dense(temp0, modules_[1].accel, temp1);
    add_dense_dense(temp1, modules_[2].accel, temp2);
    add_dense_dense(temp2, modules_[3].accel, accel_);

    angular_accel_ =
        add(div(moment, J_),
            add(add(modules_[0].angular_accel, modules_[1].angular_accel),
                add(modules_[2].angular_accel, modules_[3].angular_accel)));

    VLOG(1) << "accel(0, 0) = " << ccode(*accel_.get(0, 0));
    VLOG(1) << "accel(1, 0) = " << ccode(*accel_.get(1, 0));
    VLOG(1) << "angular_accel = " << ccode(*angular_accel_);
  }

  // Writes the physics out to the provided .py path.
  void WritePy(std::string_view py_path) {
    std::vector<std::string> result_py;

    result_py.emplace_back("#!/usr/bin/python3");
    result_py.emplace_back("");
    result_py.emplace_back("import numpy");
    result_py.emplace_back("");

    result_py.emplace_back("def swerve_physics(t, X, U_func):");
    result_py.emplace_back("    def atan2(y, x):");
    result_py.emplace_back("        if x < 0:");
    result_py.emplace_back("            return -numpy.atan2(y, x)");
    result_py.emplace_back("        else:");
    result_py.emplace_back("            return numpy.atan2(y, x)");
    result_py.emplace_back("    sin = numpy.sin");
    result_py.emplace_back("    cos = numpy.cos");
    result_py.emplace_back("    fabs = numpy.fabs");

    result_py.emplace_back("    result = numpy.empty([25, 1])");
    result_py.emplace_back("    X = X.reshape(25, 1)");
    result_py.emplace_back("    U = U_func(X)");
    result_py.emplace_back("");

    // Start by writing out variables matching each of the symbol names we use
    // so we don't have to modify the computed equations too much.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_py.emplace_back(
          absl::Substitute("    thetas$0 = X[$1, 0]", m, m * 4));
      result_py.emplace_back(
          absl::Substitute("    omegas$0 = X[$1, 0]", m, m * 4 + 2));
      result_py.emplace_back(
          absl::Substitute("    omegad$0 = X[$1, 0]", m, m * 4 + 3));
    }

    result_py.emplace_back(
        absl::Substitute("    theta = X[$0, 0]", kNumModules * 4 + 2));
    result_py.emplace_back(
        absl::Substitute("    vx = X[$0, 0]", kNumModules * 4 + 3));
    result_py.emplace_back(
        absl::Substitute("    vy = X[$0, 0]", kNumModules * 4 + 4));
    result_py.emplace_back(
        absl::Substitute("    omega = X[$0, 0]", kNumModules * 4 + 5));

    result_py.emplace_back(
        absl::Substitute("    fx = X[$0, 0]", kNumModules * 4 + 6));
    result_py.emplace_back(
        absl::Substitute("    fy = X[$0, 0]", kNumModules * 4 + 7));
    result_py.emplace_back(
        absl::Substitute("    moment = X[$0, 0]", kNumModules * 4 + 8));

    // Now do the same for the inputs.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_py.emplace_back(absl::Substitute("    Is$0 = U[$1, 0]", m, m * 2));
      result_py.emplace_back(
          absl::Substitute("    Id$0 = U[$1, 0]", m, m * 2 + 1));
    }

    result_py.emplace_back("");

    // And then write out the derivative of each state.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_py.emplace_back(
          absl::Substitute("    result[$0, 0] = omegas$1", m * 4, m));
      result_py.emplace_back(
          absl::Substitute("    result[$0, 0] = omegad$1", m * 4 + 1, m));

      result_py.emplace_back(absl::Substitute(
          "    result[$0, 0] = $1", m * 4 + 2, ccode(*modules_[m].alphas_eqn)));
      result_py.emplace_back(absl::Substitute(
          "    result[$0, 0] = $1", m * 4 + 3, ccode(*modules_[m].alphad_eqn)));
    }

    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = vx", kNumModules * 4));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = vy", kNumModules * 4 + 1));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = omega", kNumModules * 4 + 2));

    result_py.emplace_back(absl::Substitute("    result[$0, 0] = $1",
                                            kNumModules * 4 + 3,
                                            ccode(*accel_.get(0, 0))));
    result_py.emplace_back(absl::Substitute("    result[$0, 0] = $1",
                                            kNumModules * 4 + 4,
                                            ccode(*accel_.get(1, 0))));
    result_py.emplace_back(absl::Substitute(
        "    result[$0, 0] = $1", kNumModules * 4 + 5, ccode(*angular_accel_)));

    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = 0.0", kNumModules * 4 + 6));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = 0.0", kNumModules * 4 + 7));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = 0.0", kNumModules * 4 + 8));

    result_py.emplace_back("");
    result_py.emplace_back("    return result.reshape(25,)\n");

    aos::util::WriteStringToFileOrDie(py_path, absl::StrJoin(result_py, "\n"));
  }

  // Writes the physics out to the provided .cc and .h path.
  void Write(std::string_view cc_path, std::string_view h_path) {
    std::vector<std::string> result_cc;
    std::vector<std::string> result_h;

    std::string_view include_guard_stripped = h_path;
    CHECK(absl::ConsumePrefix(&include_guard_stripped,
                              absl::GetFlag(FLAGS_output_base)));
    std::string include_guard =
        absl::StrReplaceAll(absl::AsciiStrToUpper(include_guard_stripped),
                            {{"/", "_"}, {".", "_"}});

    // Write out the header.
    result_h.emplace_back(absl::Substitute("#ifndef $0_", include_guard));
    result_h.emplace_back(absl::Substitute("#define $0_", include_guard));
    result_h.emplace_back("");
    result_h.emplace_back("#include <Eigen/Dense>");
    result_h.emplace_back("");
    result_h.emplace_back("namespace frc971::control_loops::swerve {");
    result_h.emplace_back("");
    result_h.emplace_back("// Returns the derivative of our state vector");
    result_h.emplace_back("// [thetas0, thetad0, omegas0, omegad0,");
    result_h.emplace_back("//  thetas1, thetad1, omegas1, omegad1,");
    result_h.emplace_back("//  thetas2, thetad2, omegas2, omegad2,");
    result_h.emplace_back("//  thetas3, thetad3, omegas3, omegad3,");
    result_h.emplace_back("//  x, y, theta, vx, vy, omega,");
    result_h.emplace_back("//  Fx, Fy, Moment]");
    result_h.emplace_back("Eigen::Matrix<double, 25, 1> SwervePhysics(");
    result_h.emplace_back(
        "    Eigen::Map<const Eigen::Matrix<double, 25, 1>> X,");
    result_h.emplace_back(
        "    Eigen::Map<const Eigen::Matrix<double, 8, 1>> U);");
    result_h.emplace_back("");
    result_h.emplace_back("}  // namespace frc971::control_loops::swerve");
    result_h.emplace_back("");
    result_h.emplace_back(absl::Substitute("#endif  // $0_", include_guard));

    // Write out the .cc
    result_cc.emplace_back(
        absl::Substitute("#include \"$0\"", include_guard_stripped));
    result_cc.emplace_back("");
    result_cc.emplace_back("#include <cmath>");
    result_cc.emplace_back("");
    result_cc.emplace_back("namespace frc971::control_loops::swerve {");
    result_cc.emplace_back("namespace {");
    result_cc.emplace_back("");
    result_cc.emplace_back("double sign(double x) {");
    result_cc.emplace_back("  return (x > 0) - (x < 0);");
    result_cc.emplace_back("}");

    result_cc.emplace_back("");
    result_cc.emplace_back("}  // namespace");
    result_cc.emplace_back("");
    result_cc.emplace_back("Eigen::Matrix<double, 25, 1> SwervePhysics(");
    result_cc.emplace_back(
        "    Eigen::Map<const Eigen::Matrix<double, 25, 1>> X,");
    result_cc.emplace_back(
        "    Eigen::Map<const Eigen::Matrix<double, 8, 1>> U) {");
    result_cc.emplace_back("  Eigen::Matrix<double, 25, 1> result;");

    // Start by writing out variables matching each of the symbol names we use
    // so we don't have to modify the computed equations too much.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_cc.emplace_back(
          absl::Substitute("  const double thetas$0 = X($1, 0);", m, m * 4));
      result_cc.emplace_back(absl::Substitute(
          "  const double omegas$0 = X($1, 0);", m, m * 4 + 2));
      result_cc.emplace_back(absl::Substitute(
          "  const double omegad$0 = X($1, 0);", m, m * 4 + 3));
    }

    result_cc.emplace_back(absl::Substitute("  const double theta = X($0, 0);",
                                            kNumModules * 4 + 2));
    result_cc.emplace_back(
        absl::Substitute("  const double vx = X($0, 0);", kNumModules * 4 + 3));
    result_cc.emplace_back(
        absl::Substitute("  const double vy = X($0, 0);", kNumModules * 4 + 4));
    result_cc.emplace_back(absl::Substitute("  const double omega = X($0, 0);",
                                            kNumModules * 4 + 5));

    result_cc.emplace_back(
        absl::Substitute("  const double fx = X($0, 0);", kNumModules * 4 + 6));
    result_cc.emplace_back(
        absl::Substitute("  const double fy = X($0, 0);", kNumModules * 4 + 7));
    result_cc.emplace_back(absl::Substitute("  const double moment = X($0, 0);",
                                            kNumModules * 4 + 8));

    // Now do the same for the inputs.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_cc.emplace_back(
          absl::Substitute("  const double Is$0 = U($1, 0);", m, m * 2));
      result_cc.emplace_back(
          absl::Substitute("  const double Id$0 = U($1, 0);", m, m * 2 + 1));
    }

    result_cc.emplace_back("");

    // And then write out the derivative of each state.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_cc.emplace_back(
          absl::Substitute("  result($0, 0) = omegas$1;", m * 4, m));
      result_cc.emplace_back(
          absl::Substitute("  result($0, 0) = omegad$1;", m * 4 + 1, m));

      result_cc.emplace_back(absl::Substitute(
          "  result($0, 0) = $1;", m * 4 + 2, ccode(*modules_[m].alphas_eqn)));
      result_cc.emplace_back(absl::Substitute(
          "  result($0, 0) = $1;", m * 4 + 3, ccode(*modules_[m].alphad_eqn)));
    }

    result_cc.emplace_back(
        absl::Substitute("  result($0, 0) = omega;", kNumModules * 4));
    result_cc.emplace_back(
        absl::Substitute("  result($0, 0) = vx;", kNumModules * 4 + 1));
    result_cc.emplace_back(
        absl::Substitute("  result($0, 0) = vy;", kNumModules * 4 + 2));

    result_cc.emplace_back(absl::Substitute(
        "  result($0, 0) = $1;", kNumModules * 4 + 3, ccode(*angular_accel_)));
    result_cc.emplace_back(absl::Substitute("  result($0, 0) = $1;",
                                            kNumModules * 4 + 4,
                                            ccode(*accel_.get(0, 0))));
    result_cc.emplace_back(absl::Substitute("  result($0, 0) = $1;",
                                            kNumModules * 4 + 5,
                                            ccode(*accel_.get(1, 0))));

    result_cc.emplace_back(
        absl::Substitute("  result($0, 0) = 0.0;", kNumModules * 4 + 6));
    result_cc.emplace_back(
        absl::Substitute("  result($0, 0) = 0.0;", kNumModules * 4 + 7));
    result_cc.emplace_back(
        absl::Substitute("  result($0, 0) = 0.0;", kNumModules * 4 + 8));

    result_cc.emplace_back("");
    result_cc.emplace_back("  return result;");
    result_cc.emplace_back("}");
    result_cc.emplace_back("");
    result_cc.emplace_back("}  // namespace frc971::control_loops::swerve");

    aos::util::WriteStringToFileOrDie(cc_path, absl::StrJoin(result_cc, "\n"));
    aos::util::WriteStringToFileOrDie(h_path, absl::StrJoin(result_h, "\n"));
  }

  void WriteCasadiVariables(std::vector<std::string> *result_py) {
    result_py->emplace_back("    sin = casadi.sin");
    result_py->emplace_back("    sign = casadi.sign");
    result_py->emplace_back("    cos = casadi.cos");
    result_py->emplace_back("    atan2 = sin_atan2");
    result_py->emplace_back("    fmax = casadi.fmax");
    result_py->emplace_back("    fabs = casadi.fabs");

    // Start by writing out variables matching each of the symbol names we use
    // so we don't have to modify the computed equations too much.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_py->emplace_back(
          absl::Substitute("    thetas$0 = X[$1, 0]", m, m * 4));
      result_py->emplace_back(
          absl::Substitute("    omegas$0 = X[$1, 0]", m, m * 4 + 2));
      result_py->emplace_back(
          absl::Substitute("    omegad$0 = X[$1, 0]", m, m * 4 + 3));
    }

    result_py->emplace_back(
        absl::Substitute("    theta = X[$0, 0]", kNumModules * 4 + 2));
    result_py->emplace_back(
        absl::Substitute("    vx = X[$0, 0]", kNumModules * 4 + 3));
    result_py->emplace_back(
        absl::Substitute("    vy = X[$0, 0]", kNumModules * 4 + 4));
    result_py->emplace_back(
        absl::Substitute("    omega = X[$0, 0]", kNumModules * 4 + 5));

    result_py->emplace_back(
        absl::Substitute("    fx = X[$0, 0]", kNumModules * 4 + 6));
    result_py->emplace_back(
        absl::Substitute("    fy = X[$0, 0]", kNumModules * 4 + 7));
    result_py->emplace_back(
        absl::Substitute("    moment = X[$0, 0]", kNumModules * 4 + 8));

    // Now do the same for the inputs.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_py->emplace_back(
          absl::Substitute("    Is$0 = U[$1, 0]", m, m * 2));
      result_py->emplace_back(
          absl::Substitute("    Id$0 = U[$1, 0]", m, m * 2 + 1));
    }
  }

  // Writes the physics out to the provided .cc and .h path.
  void WriteCasadi(std::string_view py_path) {
    std::vector<std::string> result_py;

    // Write out the header.
    result_py.emplace_back("#!/usr/bin/python3");
    result_py.emplace_back("");
    result_py.emplace_back("import casadi");
    result_py.emplace_back("");
    result_py.emplace_back(absl::Substitute("WHEEL_RADIUS = $0", ccode(*rw_)));
    result_py.emplace_back(
        absl::Substitute("ROBOT_WIDTH = $0", ccode(*robot_width_)));
    result_py.emplace_back(absl::Substitute("CASTER = $0", ccode(*caster_)));
    result_py.emplace_back("");
    result_py.emplace_back("def sin_atan2(y, x):");
    result_py.emplace_back("    return casadi.sin(casadi.atan2(y, x))");
    result_py.emplace_back("");

    result_py.emplace_back("# Returns the derivative of our state vector");
    result_py.emplace_back("# [thetas0, thetad0, omegas0, omegad0,");
    result_py.emplace_back("#  thetas1, thetad1, omegas1, omegad1,");
    result_py.emplace_back("#  thetas2, thetad2, omegas2, omegad2,");
    result_py.emplace_back("#  thetas3, thetad3, omegas3, omegad3,");
    result_py.emplace_back("#  x, y, theta, vx, vy, omega,");
    result_py.emplace_back("#  Fx, Fy, Moment]");
    result_py.emplace_back("def swerve_physics(X, U):");
    WriteCasadiVariables(&result_py);

    result_py.emplace_back("");
    result_py.emplace_back("    result = casadi.SX.sym('result', 25, 1)");
    result_py.emplace_back("");

    // And then write out the derivative of each state.
    for (size_t m = 0; m < kNumModules; ++m) {
      result_py.emplace_back(
          absl::Substitute("    result[$0, 0] = omegas$1", m * 4, m));
      result_py.emplace_back(
          absl::Substitute("    result[$0, 0] = omegad$1", m * 4 + 1, m));

      result_py.emplace_back(absl::Substitute(
          "    result[$0, 0] = $1", m * 4 + 2, ccode(*modules_[m].alphas_eqn)));
      result_py.emplace_back(absl::Substitute(
          "    result[$0, 0] = $1", m * 4 + 3, ccode(*modules_[m].alphad_eqn)));
    }

    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = vx", kNumModules * 4 + 0));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = vy", kNumModules * 4 + 1));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = omega", kNumModules * 4 + 2));

    result_py.emplace_back(absl::Substitute("    result[$0, 0] = $1",
                                            kNumModules * 4 + 3,
                                            ccode(*accel_.get(0, 0))));
    result_py.emplace_back(absl::Substitute("    result[$0, 0] = $1",
                                            kNumModules * 4 + 4,
                                            ccode(*accel_.get(1, 0))));
    result_py.emplace_back(absl::Substitute(
        "    result[$0, 0] = $1", kNumModules * 4 + 5, ccode(*angular_accel_)));

    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = 0.0", kNumModules * 4 + 6));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = 0.0", kNumModules * 4 + 7));
    result_py.emplace_back(
        absl::Substitute("    result[$0, 0] = 0.0", kNumModules * 4 + 8));

    result_py.emplace_back("");
    result_py.emplace_back(
        "    return casadi.Function('xdot', [X, U], [result])");

    DefineVector2dFunction(
        "contact_patch_velocity",
        "# Returns the velocity of the wheel in global coordinates.",
        [](const Module &m, int dimension) {
          return ccode(*m.contact_patch_velocity.get(dimension, 0));
        },
        &result_py);
    DefineVector2dFunction(
        "wheel_ground_velocity",
        "# Returns the velocity of the wheel in steer module coordinates.",
        [](const Module &m, int dimension) {
          return ccode(*m.wheel_ground_velocity.get(dimension, 0));
        },
        &result_py);

    DefineVector2dFunction(
        "wheel_slip_velocity",
        "# Returns the difference in velocities of the wheel surface and the "
        "ground.",
        [](const Module &m, int dimension) {
          return ccode(*m.wheel_slip_velocity.get(dimension, 0));
        },
        &result_py);

    DefineScalarFunction(
        "slip_angle", "Returns the slip angle of the ith wheel",
        [](const Module &m) { return ccode(*m.slip_angle); }, &result_py);
    DefineScalarFunction(
        "slip_ratio", "Returns the slip ratio of the ith wheel",
        [](const Module &m) { return ccode(*m.slip_ratio); }, &result_py);
    DefineScalarFunction(
        "module_angular_accel",
        "Returns the angular acceleration of the robot due to the ith wheel",
        [](const Module &m) { return ccode(*m.angular_accel); }, &result_py);

    DefineVector2dFunction(
        "wheel_force",
        "Returns the force on the wheel in steer module coordinates",
        [](const Module &m, int dimension) {
          return ccode(*std::vector<RCP<const Basic>>{m.Fwx, m.Fwy}[dimension]);
        },
        &result_py);

    DefineVector2dFunction(
        "F", "Returns the force on the wheel in absolute coordinates",
        [](const Module &m, int dimension) {
          return ccode(*m.F.get(dimension, 0));
        },
        &result_py);

    DefineVector2dFunction(
        "mounting_location",
        "Returns the mounting location of wheel in robot coordinates",
        [](const Module &m, int dimension) {
          return ccode(*m.mounting_location.get(dimension, 0));
        },
        &result_py);

    DefineScalarFunction(
        "Ms", "Returns the self aligning moment of the ith wheel",
        [this](const Module &m) {
          return ccode(*(div(m.Ms, add(Jsm_, div(div(Js_, Gs_), Gs_)))));
        },
        &result_py);

    aos::util::WriteStringToFileOrDie(py_path, absl::StrJoin(result_py, "\n"));
  }

  void DefineScalarFunction(
      std::string_view name, std::string_view documentation,
      std::function<std::string(const Module &)> scalar_fn,
      std::vector<std::string> *result_py) {
    result_py->emplace_back("");
    result_py->emplace_back(absl::Substitute("# $0.", documentation));
    result_py->emplace_back(absl::Substitute("def $0(i, X, U):", name));
    WriteCasadiVariables(result_py);
    for (size_t m = 0; m < kNumModules; ++m) {
      if (m == 0) {
        result_py->emplace_back("    if i == 0:");
      } else {
        result_py->emplace_back(absl::Substitute("    elif i == $0:", m));
      }
      result_py->emplace_back(
          absl::Substitute("        return casadi.Function('$0', [X, U], [$1])",
                           name, scalar_fn(modules_[m])));
    }
    result_py->emplace_back("    raise ValueError(\"Invalid module number\")");
  }

  void DefineVector2dFunction(
      std::string_view name, std::string_view documentation,
      std::function<std::string(const Module &, int)> scalar_fn,
      std::vector<std::string> *result_py) {
    result_py->emplace_back("");
    result_py->emplace_back(absl::Substitute("# $0.", documentation));
    result_py->emplace_back(absl::Substitute("def $0(i, X, U):", name));
    WriteCasadiVariables(result_py);
    result_py->emplace_back(
        absl::Substitute("    result = casadi.SX.sym('$0', 2, 1)", name));
    for (size_t m = 0; m < kNumModules; ++m) {
      if (m == 0) {
        result_py->emplace_back("    if i == 0:");
      } else {
        result_py->emplace_back(absl::Substitute("    elif i == $0:", m));
      }
      for (int j = 0; j < 2; ++j) {
        result_py->emplace_back(absl::Substitute("        result[$0, 0] = $1",
                                                 j, scalar_fn(modules_[m], j)));
      }
    }
    result_py->emplace_back("    else:");
    result_py->emplace_back(
        "        raise ValueError(\"Invalid module number\")");
    result_py->emplace_back(absl::Substitute(
        "    return casadi.Function('$0', [X, U], [result])", name));
  }

 private:
  static constexpr uint8_t kNumModules = 4;

  Module ModulePhysics(const int m, DenseMatrix mounting_location) {
    VLOG(1) << "Solving module " << m;

    Module result;
    result.mounting_location = mounting_location;

    result.Is = symbol(absl::StrFormat("Is%u", m));
    result.Id = symbol(absl::StrFormat("Id%u", m));

    RCP<const Symbol> thetamd = symbol(absl::StrFormat("theta_md%u", m));
    RCP<const Symbol> omegamd = symbol(absl::StrFormat("omega_md%u", m));
    RCP<const Symbol> alphamd = symbol(absl::StrFormat("alpha_md%u", m));

    result.thetas = symbol(absl::StrFormat("thetas%u", m));
    result.omegas = symbol(absl::StrFormat("omegas%u", m));
    result.alphas = symbol(absl::StrFormat("alphas%u", m));

    result.thetad = symbol(absl::StrFormat("thetad%u", m));
    result.omegad = symbol(absl::StrFormat("omegad%u", m));
    result.alphad = symbol(absl::StrFormat("alphad%u", m));

    // Velocity of the module in field coordinates
    DenseMatrix robot_velocity = DenseMatrix(2, 1, {vx_, vy_});
    VLOG(1) << "robot velocity: " << robot_velocity.__str__();

    // Velocity of the contact patch in field coordinates
    DenseMatrix temp_matrix = DenseMatrix(2, 1);
    DenseMatrix temp_matrix2 = DenseMatrix(2, 1);
    result.contact_patch_velocity = DenseMatrix(2, 1);

    mul_dense_dense(R(theta_), result.mounting_location, temp_matrix);
    add_dense_dense(angle_cross(temp_matrix, omega_), robot_velocity,
                    temp_matrix2);
    mul_dense_dense(R(add(theta_, result.thetas)),
                    DenseMatrix(2, 1, {neg(caster_), integer(0)}), temp_matrix);
    add_dense_dense(temp_matrix2,
                    angle_cross(temp_matrix, add(omega_, result.omegas)),
                    result.contact_patch_velocity);

    VLOG(1);
    VLOG(1) << "contact patch velocity: "
            << result.contact_patch_velocity.__str__();

    // Relative velocity of the surface of the wheel to the ground.
    result.wheel_ground_velocity = DenseMatrix(2, 1);
    mul_dense_dense(R(neg(add(result.thetas, theta_))),
                    result.contact_patch_velocity,
                    result.wheel_ground_velocity);

    // Compute the relative velocity between the wheel surface and the ground in
    // the wheel coordinate system.
    result.wheel_slip_velocity = DenseMatrix(2, 1);
    DenseMatrix wheel_velocity =
        DenseMatrix(2, 1, {mul(rw_, result.omegad), integer(0)});
    DenseMatrix negative_wheel_ground_velocity =
        DenseMatrix(2, 1,
                    {neg(result.wheel_ground_velocity.get(0, 0)),
                     neg(result.wheel_ground_velocity.get(1, 0))});
    add_dense_dense(negative_wheel_ground_velocity, wheel_velocity,
                    result.wheel_slip_velocity);

    VLOG(1);
    VLOG(1) << "wheel ground velocity: "
            << result.wheel_ground_velocity.__str__();

    result.slip_angle = neg(atan2(result.wheel_ground_velocity.get(1, 0),
                                  result.wheel_ground_velocity.get(0, 0)));

    VLOG(1);
    VLOG(1) << "slip angle: " << result.slip_angle->__str__();

    // TODO(austin): Does this handle decel properly?
    result.slip_ratio = div(
        sub(mul(rw_, result.omegad), result.wheel_ground_velocity.get(0, 0)),
        SymEngine::max(
            {real_double(0.02), abs(result.wheel_ground_velocity.get(0, 0))}));
    VLOG(1);
    VLOG(1) << "Slip ratio " << result.slip_ratio->__str__();

    result.Fwx = simplify(mul(Cx_, result.slip_ratio));
    result.Fwy = simplify(mul(Cy_, result.slip_angle));

    // The self-aligning moment needs to flip when the module flips direction.
    result.Ms = mul(neg(result.Fwy),
                    add(div(mul(sign(result.wheel_ground_velocity.get(0, 0)),
                                contact_patch_length_),
                            integer(3)),
                        caster_));
    VLOG(1);
    VLOG(1) << "Ms " << result.Ms->__str__();
    VLOG(1);
    VLOG(1) << "Fwx " << result.Fwx->__str__();
    VLOG(1);
    VLOG(1) << "Fwy " << result.Fwy->__str__();

    // alphas = ...
    RCP<const Basic> lhms =
        mul(add(neg(wb_), mul(add(rs_, rp_), sub(integer(1), div(rb1_, rp_)))),
            mul(div(rw_, rb2_), neg(result.Fwx)));
    RCP<const Basic> lhs =
        add(add(result.Ms, div(mul(Jsm_, result.Is), Gs_)), lhms);
    RCP<const Basic> rhs = add(Jsm_, div(div(Js_, Gs_), Gs_));
    RCP<const Basic> accel_steer_eqn = simplify(div(lhs, rhs));

    VLOG(1);
    VLOG(1) << result.alphas->__str__() << " = " << accel_steer_eqn->__str__();

    lhs = sub(mul(sub(div(add(rp_, rs_), rp_), integer(1)), result.omegas),
              mul(Gd1_, mul(Gd2_, omegamd)));
    RCP<const Basic> dplanitary_eqn = sub(mul(Gd3_, lhs), result.omegad);

    lhs = sub(mul(sub(div(add(rp_, rs_), rp_), integer(1)), result.alphas),
              mul(Gd1_, mul(Gd2_, alphamd)));
    RCP<const Basic> ddplanitary_eqn = sub(mul(Gd3_, lhs), result.alphad);

    RCP<const Basic> drive_eqn = sub(
        add(mul(neg(Jdm_), div(alphamd, Gd_)), mul(Ktd_, div(result.Id, Gd_))),
        mul(neg(result.Fwx), rw_));

    VLOG(1) << "drive_eqn: " << drive_eqn->__str__();

    // Substitute in ddplanitary_eqn so we get rid of alphamd
    map_basic_basic map;
    RCP<const Set> reals = interval(NegInf, Inf, true, true);
    RCP<const Set> solve_solution = solve(ddplanitary_eqn, alphamd, reals);
    map[alphamd] = solve_solution->get_args()[1]->get_args()[0];
    VLOG(1) << "temp: " << solve_solution->__str__();
    RCP<const Basic> drive_eqn_subs = drive_eqn->subs(map);

    map.clear();
    map[result.alphas] = accel_steer_eqn;
    RCP<const Basic> drive_eqn_subs2 = drive_eqn_subs->subs(map);
    RCP<const Basic> drive_eqn_subs3 = simplify(drive_eqn_subs2);
    VLOG(1) << "drive_eqn simplified: " << drive_eqn_subs3->__str__();

    solve_solution = solve(drive_eqn_subs3, result.alphad, reals);

    RCP<const Basic> drive_accel =
        simplify(solve_solution->get_args()[1]->get_args()[0]);
    VLOG(1) << "drive_accel: " << drive_accel->__str__();

    // Compute the resulting force from the module.
    result.F = DenseMatrix(2, 1);
    mul_dense_dense(R(add(theta_, result.thetas)),
                    DenseMatrix(2, 1, {result.Fwx, result.Fwy}), result.F);

    RCP<const Basic> torque = force_cross(result.mounting_location, result.F);
    result.accel = DenseMatrix(2, 1);
    mul_dense_scalar(result.F, pow(m_, minus_one), result.accel);
    result.angular_accel = div(torque, J_);
    VLOG(1);
    VLOG(1) << "angular_accel = " << result.angular_accel->__str__();

    VLOG(1);
    VLOG(1) << "accel(0, 0) = " << result.accel.get(0, 0)->__str__();
    VLOG(1);
    VLOG(1) << "accel(1, 0) = " << result.accel.get(1, 0)->__str__();

    result.alphad_eqn = drive_accel;
    result.alphas_eqn = accel_steer_eqn;
    return result;
  }

  DenseMatrix R(const RCP<const Basic> theta) {
    return DenseMatrix(2, 2,
                       {cos(theta), neg(sin(theta)), sin(theta), cos(theta)});
  }

  DenseMatrix angle_cross(DenseMatrix a, RCP<const Basic> b) {
    return DenseMatrix(2, 1, {mul(neg(a.get(1, 0)), b), mul(a.get(0, 0), b)});
  }

  RCP<const Basic> force_cross(DenseMatrix r, DenseMatrix f) {
    return sub(mul(r.get(0, 0), f.get(1, 0)), mul(r.get(1, 0), f.get(0, 0)));
  }

  // z represents the number of teeth per gear, theta is the angle between
  // shafts(in degrees), D_02 is the pitch diameter of gear 2 and b_2 is the
  // length of the tooth of gear 2
  // returns std::pair(r_01, r_02)
  std::pair<double, double> GetBevelPitchRadius(double z1, double z2,
                                                double theta, double D_02,
                                                double b_2) {
    double gamma_1 = std::atan2(z1, z2);
    double gamma_2 = theta / 180.0 * std::numbers::pi - gamma_1;
    double R_m = D_02 / 2 / std::sin(gamma_2) - b_2 / 2;
    return std::pair(R_m * std::cos(gamma_2), R_m * std::sin(gamma_2));
  }

  Motor drive_motor_;
  Motor steer_motor_;

  RCP<const Basic> Cx_;
  RCP<const Basic> Cy_;
  RCP<const Basic> rw_;
  RCP<const Basic> m_;
  RCP<const Basic> J_;
  RCP<const Basic> Gd1_;
  RCP<const Basic> rs_;
  RCP<const Basic> rp_;
  RCP<const Basic> Gd2_;
  RCP<const Basic> rb1_;
  RCP<const Basic> rb2_;
  RCP<const Basic> Gd3_;
  RCP<const Basic> Gd_;
  RCP<const Basic> Js_;
  RCP<const Basic> Gs_;
  RCP<const Basic> wb_;
  RCP<const Basic> Jdm_;
  RCP<const Basic> Jsm_;
  RCP<const Basic> Kts_;
  RCP<const Basic> Ktd_;
  RCP<const Basic> robot_width_;
  RCP<const Basic> caster_;
  RCP<const Basic> contact_patch_length_;
  RCP<const Basic> x_;
  RCP<const Basic> y_;
  RCP<const Basic> theta_;
  RCP<const Basic> vx_;
  RCP<const Basic> vy_;
  RCP<const Basic> omega_;
  RCP<const Basic> ax_;
  RCP<const Basic> ay_;
  RCP<const Basic> atheta_;

  std::array<Module, kNumModules> modules_;

  DenseMatrix accel_;
  RCP<const Basic> angular_accel_;
};

}  // namespace frc971::control_loops::swerve

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc971::control_loops::swerve::SwerveSimulation sim;

  if (!absl::GetFlag(FLAGS_cc_output_path).empty() &&
      !absl::GetFlag(FLAGS_h_output_path).empty()) {
    sim.Write(absl::GetFlag(FLAGS_cc_output_path),
              absl::GetFlag(FLAGS_h_output_path));
  }
  if (!absl::GetFlag(FLAGS_py_output_path).empty()) {
    sim.WritePy(absl::GetFlag(FLAGS_py_output_path));
  }
  if (!absl::GetFlag(FLAGS_casadi_py_output_path).empty()) {
    sim.WriteCasadi(absl::GetFlag(FLAGS_casadi_py_output_path));
  }

  return 0;
}
