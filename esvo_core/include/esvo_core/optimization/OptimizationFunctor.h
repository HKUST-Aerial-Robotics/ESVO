#ifndef ESVO_CORE_OPTIMIZATION_OPTIMIZATIONFUNCTOR_H
#define ESVO_CORE_OPTIMIZATION_OPTIMIZATIONFUNCTOR_H

#include <stdlib.h>
#include <Eigen/Eigen>

namespace esvo_core
{
namespace optimization
{

/**
 * Generic functor base for use with the Eigen-nonlinear optimization
 * toolbox. Please refer to the Eigen-documentation for further information.
 */
template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
struct OptimizationFunctor
{
  /** undocumented */
  typedef _Scalar Scalar;
  /** undocumented */
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  /** undocumented */
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  /** undocumented */
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  /** undocumented */
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  /** undocumented */
  const int m_inputs;
  /** undocumented */
  int m_values;

  /** undocumented */
  OptimizationFunctor() :
    m_inputs(InputsAtCompileTime),
    m_values(ValuesAtCompileTime) {}
  /** undocumented */
  OptimizationFunctor(int inputs, int values) :
    m_inputs(inputs),
    m_values(values) {}

  /** undocumented */
  int inputs() const
  {
    return m_inputs;
  }
  /** undocumented */
  int values() const
  {
    return m_values;
  }

  void resetNumberValues( int values )
  {
    m_values = values;
  }
};

}
}
#endif //ESVO_CORE_OPTIMIZATION_OPTIMIZATIONFUNCTOR_H
