

#ifndef ONBOARD_BASE_MACROS_H_
#define ONBOARD_BASE_MACROS_H_

#ifndef ASSERT_OK
#define ASSERT_OK(E) ASSERT_TRUE(E.ok())
#endif

#ifndef EXPECT_OK
#define EXPECT_OK(E) EXPECT_TRUE(E.ok())
#endif
#ifndef EXPECT_NOT_OK
#define EXPECT_NOT_OK(E) EXPECT_FALSE(E.ok())
#endif

#undef LIKELY
#undef UNLIKELY
#define LIKELY(x) (__builtin_expect((x), 1))
#define UNLIKELY(x) (__builtin_expect((x), 0))

#endif  // ONBOARD_BASE_MACROS_H_
