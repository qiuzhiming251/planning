

#ifndef ONBOARD_GLOBAL_SINGLETON_H_
#define ONBOARD_GLOBAL_SINGLETON_H_

#if defined(DISALLOW_COPY_AND_ASSIGN)
#undef DISALLOW_COPY_AND_ASSIGN
#endif

#if defined(DECLARE_SINGLETON)
#undef DECLARE_SINGLETON
#endif

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
//
// For disallowing only assign or copy, write the code directly, but declare
// the intent in a comment, for example:
// void operator=(const TypeName&);  // DISALLOW_ASSIGN
// Note, that most uses of DISALLOW_ASSIGN and DISALLOW_COPY are broken
// semantically, one should either use disallow both or neither. Try to
// avoid these in new code.

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;      \
  TypeName& operator=(const TypeName&) = delete

#define DECLARE_SINGLETON(classname)              \
 public:                                          \
  static classname* Instance() {                  \
    static classname* instance = new classname(); \
    return instance;                              \
  }                                               \
                                                  \
  DISALLOW_COPY_AND_ASSIGN(classname);            \
                                                  \
 private:                                         \
  classname();

#endif  // ONBOARD_GLOBAL_SINGLETON_H_
