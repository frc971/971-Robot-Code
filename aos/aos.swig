%module aos

%include "std_string.i";
%include "stdint.i";

%{
#include <time.h>
#include "aos/common/time.h"
#include "aos/common/queue.h"
static_assert(sizeof(int) == sizeof(clockid_t),
              "Sizeof clockid_t has changed.");
%}

%apply int { clockid_t };

// Makes the constructors from c++ types public so that this code can be called
// from other places.
%typemap(javabody) SWIGTYPE %{
  private long swigCPtr;
  protected boolean swigCMemOwn;

  public $javaclassname(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}

%rename(add) operator+;
%rename(subtract) operator-;
%rename(multiply) operator*;
%rename(divide) operator/;
%rename(modulo) operator%;
%rename(equalTo) operator==;
%rename(notEqual) operator!=;
%rename(greaterThan) operator>;
%rename(lessThan) operator<;
%rename(leq) operator<=;
%rename(geq) operator>=;

%include "aos/common/time.h"
%include "aos/common/queue.h"
