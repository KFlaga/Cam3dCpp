#pragma once

#include "PreReqs.h"

namespace cam3d
{
	template<typename T>
	struct Vector2
	{
	public:
		T x;
		T y;

		constexpr Vector2() : x{ 0 }, y{ 0 } { }
		constexpr Vector2(T x_, T y_) : x{ x_ }, y{ y_ } { }

		Vector2& operator+=(Vector2 a)
		{
			x += a.x;
			y += a.y;
		}

		Vector2& operator-=(Vector2 a)
		{
			x += a.x;
			y += a.y;
		}

		Vector2& operator*=(double s)
		{
			x *= s;
			y *= s;
		}

		Vector2& operator/=(double s)
		{
			x /= s;
			y /= s;
		}

		double dot(Vector2 v)
		{
			return x * v.x + y * v.y;
		}

		double cross(Vector2 v)
		{
			return x * v.y - y * v.x;
		}

		double lengthSquared()
		{
			return x * x + y * y;
		}

		double length()
		{
			return std::sqrt(x * x + y * y);
		}

		void normalise()
		{
			double len = length();
			if (len > 0)
			{
				x /= len;
				y /= len;
			}
		}

		double distanceToSquared(Vector2 v)
		{
			return (x - v.x) * (x - v.x) + (y - v.y) * (y - v.y);
		}

		double distanceTo(Vector2 v)
		{
			return std::sqrt(distanceToSquared(v));
		}

		// Returns value in radians
		double angleTo(Vector2 v)
		{
			return std::asin(sinusTo(v));
		}

		// Returns value in radians. Assumes both vector are normalized
		double angleToNormalized(Vector2 v)
		{
			return std::asin(cross(v));
		}

		// Returns sinus of angle to v, value in radians
		double sinusTo(Vector2 v)
		{
			return cross(v) / std::sqrt(lengthSquared() * v.lengthSquared());
		}

		// Returns sinus of angle to v, value in radians. Assumes both vector are normalized
		public double sinusToNormalized(Vector2 v)
		{
			return cross(v);
		}
		// Returns cosinus of angle to v, value in radians
		public double cosinusTo(Vector2 v)
		{
			return dot(v) / std::sqrt(lengthSquared() * v.lengthSquared());
		}

		// Returns cosinus of angle to v, value in radians. Assumes both vector are normalized
		public double cosinusToNormalized(Vector2 v)
		{
			return dot(v);
		}

		template<typename T2>
		operator Vector2<T2>() const
		{
			return Vector2<T2>{x, y};
		}

		template<typename T2>
		constexpr Vector2(const Vector2<T2>& v) : x{v.x}, y{v.y} { }
	};

	template<typename T>
	constexpr Vector2<T> operator+(Vector2<T> a, Vector2<T> b)
	{
		return Vector2<T>{ a.x + b.x, a.y + b.y };
	}

	template<typename T>
	constexpr Vector2<T> operator-(Vector2<T> a, Vector2<T> b)
	{
		return Vector2<T>{ a.x - b.x, a.y - b.y };
	}

	template<typename T>
	constexpr Vector2<T> operator*(Vector2<T> a, Vector2<T> b)
	{
		return Vector2<T>{ a.x * b.x, a.y * b.y };
	}

	template<typename T>
	constexpr Vector2<T> operator*(Vector2<T> a, Vector2<T> b)
	{
		return Vector2<T>{ a.x / b.x, a.y / b.y };
	}

	template<typename T, typename ScalarT>
	constexpr Vector2<T> operator*(Vector2<T> a, ScalarT s)
	{
		return Vector2<T>{ a.x * s, a.y * s };
	}

	template<typename T, typename ScalarT>
	constexpr Vector2<T> operator*(ScalarT s, Vector2<T> a)
	{
		return Vector2<T>{ a.x * s, a.y * s };
	}

	template<typename T, typename ScalarT>
	constexpr Vector2<T> operator/(Vector2<T> a, ScalarT s)
	{
		return Vector2<T>{ a.x / s, a.y / s };
	}

	template<typename T, typename ScalarT>
	constexpr Vector2<T> operator/(ScalarT s, Vector2<T> a)
	{
		return Vector2<T>{ a.x / s, a.y / s };
	}

	template<typename T>
	constexpr bool operator==(Vector2<T> a, Vector2<T> b)
	{
		return a.x == b.x && a.y == b.y;
	}

	template<typename T>
	constexpr bool operator!=(Vector2<T> a, Vector2<T> b)
	{
		return a.x != b.x && a.y != b.y;
	}

	template<typename T>
	constexpr bool operator>(Vector2<T> a, Vector2<T> b)
	{
		return a.lengthSquared() > b.lengthSquared();
	}

	template<typename T>
	constexpr bool operator<(Vector2<T> a, Vector2<T> b)
	{
		return a.lengthSquared() < b.lengthSquared();
	}

	typedef Vector2<Int> Vector2i;
	typedef Vector2<UInt> Vector2u;
	typedef Vector2<double> Vector2f;
}

