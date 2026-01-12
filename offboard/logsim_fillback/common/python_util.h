#ifndef COMMON_PYTHON_HELPER_H
#define COMMON_PYTHON_HELPER_H

#include <functional>
#include <memory>
#include "Python.h"
namespace worldview {
namespace util {

class PyObjWrapper {
 public:
  using py_deleter_t = std::function<void(PyObject*)>;
  PyObjWrapper(PyObject* p) : ptr_(p, [](PyObject* p) { Py_XDECREF(p); }) {}
  PyObject* Get() const { return ptr_.get(); }

  operator PyObject*() const { return Get(); }
  operator bool() const { return Get(); }

 private:
  std::unique_ptr<PyObject, py_deleter_t> ptr_;
};

}  // namespace util
}  // namespace worldview

#endif  // COMMON_PYTHON_HELPER_H