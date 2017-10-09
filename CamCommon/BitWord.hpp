#pragma once

#include "PreReqs.hpp"
#include <algorithm>
#include <cstring>

namespace cam3d
{
	class HammingLookup32
	{
		HammingLookup32() = delete;

		static constexpr size_t wordBitsSize = ((size_t)std::numeric_limits<uint16_t>::max()) + 1;
		static uint32_t* createWordBitsLut()
		{
			static uint32_t wordBits[wordBitsSize];
			size_t x;
			uint32_t count = 0u;
			for (size_t i = 0; i < wordBitsSize; ++i)
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
			uint64_t x;
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

	template<size_t length, size_t bits, typename uint_type, typename HammingLookup>
	struct BitWord
	{
		static constexpr size_t lengthInWords = length;
		static constexpr size_t bitSizeOfWord = bits;
		using uint_t = uint_type;

		uint_t bytes[length];
		BitWord()
		{
			std::memset(bytes, 0, sizeof(bytes));
		}

		BitWord(uint_t* bytes_)
		{
			std::memcpy(bytes, bytes_, sizeof(bytes));
		}

	private:
		template<size_t i, bool = false>
		struct helper
		{
			static uint_t getOnesCount(const uint_t* b1, const uint_t* b2)
			{
				return HammingLookup::getOnesCount(b1[i] ^ b2[i]) + helper<i + 1>::getOnesCount(b1, b2);
			}
		};

		template<bool dummy>
		struct helper<length, dummy>
		{
			static uint_t getOnesCount(const uint_t*, const uint_t*)
			{
				return 0;
			}
		};

	public:
		inline uint_t getHammingDistance(const BitWord& bw) const
		{
			return helper<0>::getOnesCount(bytes, bw.bytes);
		}
	};

	template<size_t length>
	using BitWord32 = BitWord<length, 32, uint32_t, HammingLookup32>;
	template<size_t length>
	using BitWord64 = BitWord<length, 64, uint64_t, HammingLookup64>;
}
