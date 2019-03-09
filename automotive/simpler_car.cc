#include "drake/automotive/simpler_car.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/common/cond.h"
#include "drake/common/default_scalars.h"
#include "drake/common/double_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

namespace automotive {

namespace {  // Local helper function.

// Obtain our continuous state from a context.
template <typename T>
const SimplerCarState<T>& get_state(const systems::Context<T>& context) {
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const SimplerCarState<T>* const state =
      dynamic_cast<const SimplerCarState<T>*>(&context_state);
  DRAKE_DEMAND(state);
  return *state;
}

// Obtain our input from a context.
template <typename T>
const SimplerCarInput<T>& get_input(const SimplerCar<T>* simpler_car,
                                    const systems::Context<T>& context) {
  const SimplerCarInput<T>* const input =
      simpler_car->template EvalVectorInput<SimplerCarInput>(context, 0);
  DRAKE_DEMAND(input);
  return *input;
}

// Obtain our parameters from a context.
template <typename T>
const SimplerCarParams<T>& get_params(const systems::Context<T>& context) {
  const SimplerCarParams<T>& params = dynamic_cast<const SimplerCarParams<T>&>(
      context.get_numeric_parameter(0));
  return params;
}

}  // namespace

template <typename T>
SimplerCar<T>::SimplerCar()
    : systems::LeafSystem<T>(systems::SystemTypeTag<automotive::SimplerCar>{}) {
  this->DeclareVectorInputPort(SimplerCarInput<T>());
  this->DeclareVectorOutputPort(&SimplerCar::CalcStateOutput);
  this->DeclareContinuousState(SimplerCarState<T>());
  this->DeclareNumericParameter(SimplerCarParams<T>());
}

template <typename T>
template <typename U>
SimplerCar<T>::SimplerCar(const SimplerCar<U>&) : SimplerCar() {}

template <typename T>
const systems::OutputPort<T>& SimplerCar<T>::state_output() const {
  return this->get_output_port(0);
}

template <typename T>
void SimplerCar<T>::CalcStateOutput(const systems::Context<T>& context,
                                    SimplerCarState<T>* output) const {
  const SimplerCarState<T>& state = get_state(context);
  output->set_value(state.get_value());
}

template <typename T>
void SimplerCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters.
  const SimplerCarParams<T>& params =
      this->template GetNumericParameter<SimplerCarParams>(context, 0);

  // Obtain the state.
  const SimplerCarState<T>& state = get_state(context);

  // Obtain the input.
  const SimplerCarInput<T>* const input =
      this->template EvalVectorInput<SimplerCarInput>(context, 0);
  DRAKE_ASSERT(input);

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>& vector_derivatives =
      derivatives->get_mutable_vector();
  SimplerCarState<T>* const rates =
      dynamic_cast<SimplerCarState<T>*>(&vector_derivatives);
  DRAKE_ASSERT(rates);

  ImplCalcTimeDerivatives(params, state, *input, rates);
}

template <typename T>
void SimplerCar<T>::ImplCalcTimeDerivatives(const SimplerCarParams<T>& params,
                                            const SimplerCarState<T>& state,
                                            const SimplerCarInput<T>& input,
                                            SimplerCarState<T>* rates) const {
  using std::cos;
  using std::sin;
  using std::tan;

  const T& v{input.velocity()};
  const T& theta{state.heading()};
  const T& L{params.wheelbase()};

  rates->set_x(v * cos(theta));
  rates->set_y(v * sin(theta));
  rates->set_heading(v / L * tan(input.steering()));
}

}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in simpler_car.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::SimplerCar)
