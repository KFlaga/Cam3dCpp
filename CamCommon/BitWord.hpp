#pragma once

#include "PreReqs.h"
#include <algorithm>

namespace cam3d
{
	class HammingLookup32
	{
		HammingLookup32() = delete;

		static constexpr size_t wordBitsSize = ((uint64_t)std::numeric_limits<uint16_t>::max()) + 1;
		static uint32_t* createWordBitsLut()
		{
			static uint32_t wordBits[wordBitsSize];
			int x;
			uint32_t count = 0u;
			for (int i = 0; i < wordBitsSize; ++i)
			{
				x = i;
				for (count = 0u; x > 0; ++count)
					x &= x - 1;

				wordBits[i] = count;
			}
			return wordBits;
		}

	public:
		static inline uint32_t getOnesCount(uint32_t i)
		{
			static uint32_t* wordBits = createWordBitsLut();
			return (wordBits[i & 0x0000FFFF] + wordBits[i >> 16]);
		}
	};

	class HammingLookup64
	{
		HammingLookup64() = delete;

		static constexpr uint64_t wordBitsSize = ((uint64_t)std::numeric_limits<uint16_t>::max()) + 1;
		static uint64_t* createWordBitsLut()
		{
			static uint64_t wordBits[wordBitsSize];
			int64_t x;
			uint64_t count = 0u;
			for (uint64_t i = 0u; i < wordBitsSize; ++i)
			{
				x = i;
				for (count = 0u; x > 0; ++count)
					x &= x - 1;

				wordBits[i] = count;
			}
			return wordBits;
		}

	public:
		static inline uint64_t getOnesCount(uint64_t i)
		{
			static uint64_t* wordBits = createWordBitsLut();
			return (wordBits[i & 0xFFFF] + wordBits[(i >> 16) & 0xFFFF] + wordBits[(i >> 32) & 0xFFFF] + wordBits[(i >> 48) & 0xFFFF]);
		}
	};

	template<size_t length>
	struct BitWord32
	{
		static constexpr size_t wordLength = length;
		typedef uint32_t uintType;
		
		uint32_t bytes[length];

		BitWord32()
		{
			std::memset(bytes, 0, sizeof(bytes));
		}

		BitWord32(uint32_t* bytes_)
		{
			std::memcpy(bytes, bytes_, sizeof(bytes));
		}

	private:
		template<size_t i>
		struct helper
		{
			static uint32_t getOnesCount(uint32_t* b1, uint32_t* b2)
			{
				return HammingLookup32::getOnesCount(b1[i] ^ b2[i]) + helper<i + 1>::getOnesCount(b1, b2);
			}
		};

		template<>
		struct helper<length>
		{
			static uint32_t getOnesCount(uint32_t* b1, uint32_t* b2)
			{
				return 0;
			}
		};

	public:
		inline uint32_t getHammingDistance(BitWord32& bw)
		{
			return helper<0>::getOnesCount(bytes, bw.bytes);
		}
	};

	template<size_t length>
	struct BitWord64
	{
		static constexpr size_t wordLength = length;
		typedef uint64_t uintType;
		
		uint64_t bytes[length];

		BitWord64()
		{
			std::memset(bytes, 0, sizeof(bytes));
		}

		BitWord64(uint64_t* bytes_)
		{
			std::memcpy(bytes, bytes_, sizeof(bytes));
		}

	private:
		template<size_t i>
		struct helper
		{
			static uint64_t getOnesCount(uint64_t* b1, uint64_t* b2)
			{
				return HammingLookup64::getOnesCount(b1[i] ^ b2[i]) + helper<i + 1>::getOnesCount(b1, b2);
			}
		};

		template<>
		struct helper<length>
		{
			static uint64_t getOnesCount(uint64_t* b1, uint64_t* b2)
			{
				return 0;
			}
		};

	public:
		inline uint64_t getHammingDistance(BitWord64& bw)
		{
			return helper<0>::getOnesCount(bytes, bw.bytes);
		}
	};
}
