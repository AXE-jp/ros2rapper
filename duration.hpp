#ifndef DURATION_HPP
#define DURATION_HPP

#include "timestamp.hpp"

#define DURATION_SIZE	TIMESTAMP_SIZE

typedef timestamp duration;

#define DURATION_ZERO		{0x00000000, 0x00000000}
#define DURATION_INFINITE	{0x7fffffff, 0xffffffff}

#endif // !DURATION_HPP
