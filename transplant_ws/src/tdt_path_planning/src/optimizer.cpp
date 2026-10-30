#include "tdt_path_planning/core.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#ifndef OSQP_ENABLE_PROFILING
#error "This baseline requires OSQP profiling for enforced solver time limits"
#endif

namespace tdt {
namespace {
using Coefficients = Eigen::Matrix<double, 6, 2>;
using Box = Eigen::Vector4d;

Eigen::Matrix<double, 1, 6> basis(double parameter, int derivative, double duration) {
  Eigen::Matrix<double, 1, 6> values = Eigen::Matrix<double, 1, 6>::Zero();
  for (int power = derivative; power < 6; ++power) {
    double factor = 1;
    for (int order = 0; order < derivative; ++order) factor *= power - order;
    values[power] = factor * std::pow(parameter, power - derivative) / std::pow(duration, derivative);
  }
  return values;
}

Box corridor(const Environment &environment, const Point &point, double range) {
  const double available = environment.clearance(point) - environment.config().requiredClearance();
  if (available <= 0) throw std::runtime_error("unsafe corridor anchor");
  const double half = std::min(range, available * 0.5 / std::sqrt(2.0));
  return {point.x() - half, point.y() - half, point.x() + half, point.y() + half};
}

std::vector<Coefficients> solve(const Path &anchors, const std::vector<Box> &boxes,
  const std::vector<double> &times, const Config &config, Clock::time_point deadline, int &solves) {
  const int segments = static_cast<int>(times.size());
  const int variables = 6 * segments;
  const int rows = 4 + 4 * (segments - 1);
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variables, variables);
  for (int segment = 0; segment < segments; ++segment)
    for (int first = 3; first < 6; ++first)
      for (int second = 3; second < 6; ++second) {
        const double factor_first = first * (first - 1) * (first - 2);
        const double factor_second = second * (second - 1) * (second - 2);
        hessian(6 * segment + first, 6 * segment + second) =
          2 * factor_first * factor_second / ((first + second - 5) * std::pow(times[segment], 5));
      }
  const double hessian_scale = hessian.cwiseAbs().maxCoeff();
  if (!hessian.allFinite() || hessian_scale <= 0) throw std::runtime_error("invalid QP time scale");
  hessian /= hessian_scale;
  hessian.diagonal().array() += 1e-10;
  Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(rows, variables);
  Eigen::MatrixXd low = Eigen::MatrixXd::Zero(rows, 2);
  Eigen::MatrixXd high = Eigen::MatrixXd::Zero(rows, 2);
  int row = 0;
  constraints.block<1, 6>(row, 0) = basis(0, 0, times.front());
  low.row(row) = high.row(row) = anchors.front().transpose();
  ++row;
  constraints.block<1, 6>(row++, 0) = basis(0, 1, times.front());
  constraints.block<1, 6>(row, variables - 6) = basis(1, 0, times.back());
  low.row(row) = high.row(row) = anchors.back().transpose();
  ++row;
  constraints.block<1, 6>(row++, variables - 6) = basis(1, 1, times.back());
  for (int segment = 0; segment + 1 < segments; ++segment) {
    constraints.block<1, 6>(row, 6 * segment) = basis(1, 0, times[segment]);
    constraints.block<1, 6>(row, 6 * (segment + 1)) = -basis(0, 0, times[segment + 1]);
    ++row;
    constraints.block<1, 6>(row, 6 * segment) = basis(1, 0, times[segment]);
    low.row(row) = boxes[segment + 1].head<2>().transpose();
    high.row(row) = boxes[segment + 1].tail<2>().transpose();
    ++row;
    for (int derivative = 1; derivative <= 2; ++derivative) {
      constraints.block<1, 6>(row, 6 * segment) = basis(1, derivative, times[segment]);
      constraints.block<1, 6>(row, 6 * (segment + 1)) = -basis(0, derivative, times[segment + 1]);
      ++row;
    }
  }
  for (int index = 0; index < rows; ++index) {
    const double scale = constraints.row(index).norm();
    constraints.row(index) /= scale;
    low.row(index) /= scale;
    high.row(index) /= scale;
  }
  Eigen::SparseMatrix<double> sparse_hessian = hessian.sparseView();
  Eigen::SparseMatrix<double> sparse_constraints = constraints.sparseView();
  Eigen::MatrixXd solutions(variables, 2);
  for (int axis = 0; axis < 2; ++axis) {
    const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
    if (remaining <= 0) throw std::runtime_error("QP deadline exceeded");
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setPolish(true);
    solver.settings()->setMaxIteration(config.qp_max_iterations);
    solver.settings()->setAbsoluteTolerance(1e-7);
    solver.settings()->setRelativeTolerance(1e-7);
    solver.settings()->getSettings()->time_limit = std::min(remaining, config.qp_budget_ms / 1000.0);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
    Eigen::VectorXd lower = low.col(axis);
    Eigen::VectorXd upper = high.col(axis);
    solver.data()->setNumberOfVariables(variables);
    solver.data()->setNumberOfConstraints(rows);
    if (!solver.data()->setHessianMatrix(sparse_hessian) || !solver.data()->setGradient(gradient) ||
      !solver.data()->setLinearConstraintsMatrix(sparse_constraints) ||
      !solver.data()->setLowerBound(lower) || !solver.data()->setUpperBound(upper) || !solver.initSolver())
      throw std::runtime_error("QP setup failed");
    ++solves;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError || solver.getStatus() != OsqpEigen::Status::Solved)
      throw std::runtime_error("QP did not solve accurately");
    solutions.col(axis) = solver.getSolution();
    const Eigen::VectorXd values = constraints * solutions.col(axis);
    if (!solutions.col(axis).allFinite() || (lower - values).maxCoeff() > 1e-5 ||
      (values - upper).maxCoeff() > 1e-5) throw std::runtime_error("QP residual rejected");
  }
  std::vector<Coefficients> result(segments);
  for (int segment = 0; segment < segments; ++segment)
    result[segment] = solutions.block<6, 2>(6 * segment, 0);
  for (int segment = 0; segment + 1 < segments; ++segment)
    for (int derivative = 0; derivative <= 2; ++derivative)
      if ((basis(1, derivative, times[segment]) * result[segment] -
        basis(0, derivative, times[segment + 1]) * result[segment + 1]).norm() > 1e-4)
        throw std::runtime_error("physical continuity residual rejected");
  if ((result.front().row(0).transpose() - anchors.front()).norm() > 1e-5 ||
    (basis(1, 0, times.back()) * result.back() - anchors.back().transpose()).norm() > 1e-5 ||
    (basis(0, 1, times.front()) * result.front()).norm() > 1e-5 ||
    (basis(1, 1, times.back()) * result.back()).norm() > 1e-5)
    throw std::runtime_error("endpoint residual rejected");
  return result;
}

bool sampleCurve(const Environment &environment, const Coefficients &coefficients,
  Clock::time_point deadline, Path &output, int &checked) {
  const auto evaluate = [&](double parameter) -> Point {
    return (basis(parameter, 0, 1) * coefficients).transpose();
  };
  double second_bound = 0;
  for (int power = 2; power < 6; ++power)
    second_bound += power * (power - 1) * coefficients.row(power).norm();
  std::function<bool(double, double, int)> subdivide = [&](double first, double last, int depth) {
    if (++checked > environment.config().max_check_samples || Clock::now() >= deadline)
      throw std::runtime_error("curve checking budget exceeded");
    const Point start = evaluate(first);
    const Point end = evaluate(last);
    const double error_bound = second_bound * (last - first) * (last - first) / 8.0;
    if ((end - start).norm() > std::min(environment.config().max_chord, environment.geometry().resolution) ||
      error_bound > environment.config().curve_error) {
      if (depth >= 30) throw std::runtime_error("curve subdivision limit exceeded");
      const double middle = (first + last) * 0.5;
      const bool left = subdivide(first, middle, depth + 1);
      const bool right = subdivide(middle, last, depth + 1);
      return left && right;
    }
    output.push_back(end);
    return environment.segmentSafe(start, end, error_bound, deadline);
  };
  output.push_back(evaluate(0));
  return subdivide(0, 1, 0);
}

Path resample(const Path &path, double spacing, int limit) {
  Path output{path.front()};
  double remaining = spacing;
  for (size_t index = 1; index < path.size(); ++index) {
    Point start = path[index - 1];
    double length = (path[index] - start).norm();
    while (length >= remaining && length > 1e-10) {
      start += (path[index] - start) * (remaining / length);
      output.push_back(start);
      if (output.size() > static_cast<size_t>(limit)) throw std::runtime_error("output sample budget exceeded");
      length = (path[index] - start).norm();
      remaining = spacing;
    }
    remaining -= length;
  }
  if ((output.back() - path.back()).norm() > 1e-9) output.push_back(path.back());
  output.back() = path.back();
  return output;
}
}

Result plan(const Environment &environment, const Point &start, const Point &goal) {
  Result result;
  const auto begin = Clock::now();
  const auto &config = environment.config();
  const auto deadline = begin + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double, std::milli>(config.total_budget_ms));
  try {
    const auto search_deadline = std::min(deadline, begin + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double, std::milli>(config.search_budget_ms)));
    result.raw = search(environment, start, goal, search_deadline);
    result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    result.simplified = simplify(environment, result.raw, deadline);
    if (Clock::now() >= deadline) throw std::runtime_error("simplification deadline exceeded");
    Path anchors = result.simplified;
    if (anchors.size() > static_cast<size_t>(config.max_control_points))
      throw std::runtime_error("control point budget exceeded");
    const auto optimization_begin = Clock::now();
    std::vector<Box> boxes;
    for (const auto &anchor : anchors) boxes.push_back(corridor(environment, anchor, config.corridor_range));
    boxes.front() = {start.x(), start.y(), start.x(), start.y()};
    boxes.back() = {goal.x(), goal.y(), goal.x(), goal.y()};
    for (int round = 0; round < config.max_refine_rounds; ++round) {
      if (Clock::now() >= deadline) throw std::runtime_error("planning deadline exceeded");
      result.rounds = round + 1;
      std::vector<double> times;
      for (size_t index = 1; index < anchors.size(); ++index) {
        const double length = (anchors[index] - anchors[index - 1]).norm();
        if (length < 1e-8) throw std::runtime_error("degenerate optimization segment");
        const double switch_distance = config.allocation_speed * config.allocation_speed / config.allocation_acceleration;
        const double duration = length < switch_distance ? 2 * std::sqrt(length / config.allocation_acceleration) :
          length / config.allocation_speed + config.allocation_speed / config.allocation_acceleration;
        if (!std::isfinite(duration) || duration <= 0) throw std::runtime_error("invalid segment duration");
        times.push_back(std::max(config.min_segment_time, duration));
      }
      const auto coefficients = solve(anchors, boxes, times, config, deadline, result.qp_solves);
      Path dense;
      std::vector<size_t> bad;
      int checked = 0;
      for (size_t segment = 0; segment < coefficients.size(); ++segment) {
        Path samples;
        if (!sampleCurve(environment, coefficients[segment], deadline, samples, checked)) bad.push_back(segment);
        dense.insert(dense.end(), samples.begin() + (segment ? 1 : 0), samples.end());
      }
      if (bad.empty()) {
        dense.front() = start;
        dense.back() = goal;
        Path output = resample(dense, config.output_spacing, config.max_check_samples);
        if (environment.pathSafe(output, deadline) && Clock::now() < deadline) {
          result.path = std::move(output);
          result.coefficients = coefficients;
          result.segment_times = times;
          result.corridors = boxes;
          result.success = true;
          result.reason = "success";
          break;
        }
        for (size_t segment = 0; segment < coefficients.size(); ++segment) bad.push_back(segment);
      }
      if (round % 2 == 0) {
        for (auto iterator = bad.rbegin(); iterator != bad.rend(); ++iterator) {
          if (anchors.size() >= static_cast<size_t>(config.max_control_points))
            throw std::runtime_error("control point budget exceeded");
          const size_t segment = *iterator;
          const Point middle = (anchors[segment] + anchors[segment + 1]) * 0.5;
          if (!environment.segmentSafe(anchors[segment], middle, 0.0, deadline) ||
            !environment.segmentSafe(middle, anchors[segment + 1], 0.0, deadline))
            throw std::runtime_error("unsafe reference midpoint");
          anchors.insert(anchors.begin() + segment + 1, middle);
          boxes.insert(boxes.begin() + segment + 1, corridor(environment, middle, config.corridor_range));
        }
      } else {
        for (const auto segment : bad)
          for (const auto index : {segment, segment + 1}) {
            const Point half = (boxes[index].tail<2>() - boxes[index].head<2>()) * 0.25;
            boxes[index] = {anchors[index].x() - half.x(), anchors[index].y() - half.y(),
              anchors[index].x() + half.x(), anchors[index].y() + half.y()};
          }
      }
    }
    result.optimize_ms = std::chrono::duration<double, std::milli>(Clock::now() - optimization_begin).count();
    if (!result.success) result.reason = "collision refinement exhausted";
  } catch (const std::exception &error) {
    result.reason = error.what();
    result.path.clear();
    result.success = false;
  }
  result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
  return result;
}

}
