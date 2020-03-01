#ifndef MATRIXMATH_H
#define MATRIXMATH_H

#include <stdint.h>
#include <math.h>
#include <type_traits>

#define MIN(x,y) (((x)<(y))?(x):(y))

template <uint32_t N, uint32_t M>
class Mat {
public:
    float d[N*M];

    Mat() {}
    Mat(float v) {
        fill(v);
    }

    inline float& at(uint32_t row, uint32_t col) {
        return d[row*M+col];
    }
    inline float at(uint32_t row, uint32_t col) const {
        return d[row*M+col];
    }

    // Some Template Metaprogramming nonsense, vectors are directly indexable
    template<uint32_t cols = M>
    typename std::enable_if<cols == 1, float&>::type operator[](int i) {
        return d[i];
    }
    template<uint32_t cols = M>
    typename std::enable_if<cols == 1, float>::type operator[](int i) const {
        return d[i];
    }


    void fill(float val)
    {
        for(uint32_t i = 0; i < N*M; ++i) {
            d[i] = val;
        }
    }

    void operator+=(float val) {
        for(uint32_t i = 0; i < MIN(N,M); ++i) {
            at(i,i) += val;
        }
    }

    void operator-=(float val) {
        for(uint32_t i = 0; i < MIN(N,M); ++i) {
            at(i,i) -= val;
        }
    }

    void operator/=(float val) {
        for(uint32_t i = 0; i < N*M; ++i) {
            d[i] /= val;
        }
    }
    Mat<N,M> operator/(float val) const {
        Mat<N,M> ret(*this);
        ret /= val;
        return ret;
    }

    void operator*=(float val) {
        for(uint32_t i = 0; i < N*M; ++i) {
            d[i] *= val;
        }
    }
    Mat<N,M> operator*(float val) const {
        Mat<N,M> ret(*this);
        ret *= val;
        return ret;
    }

    void operator+=(const Mat<N,M>& m2) {
        for(uint32_t i = 0; i < N*M; ++i) {
            d[i] += m2.d[i];
        }
    }
    Mat<N,M> operator+(const Mat<N,M>& m2) const {
        Mat<N,M> ret(*this);
        ret += m2;
        return ret;
    }

    void operator-=(const Mat<N,M>& m2) {
        for(uint32_t i = 0; i < N*M; ++i) {
            d[i] -= m2.d[i];
        }
    }
    Mat<N,M> operator-(const Mat<N,M>& m2) const {
        Mat<N,M> ret(*this);
        ret -= m2;
        return ret;
    }

    Mat<N,M> operator-() const {
        Mat<N,M> ret;
        for(uint32_t i = 0; i < N*M; ++i) {
            ret.d[i] = -d[i];
        }
        return ret;
    }

    Mat<M,N> transpose() const {
        Mat<M,N> ret;
        for(uint32_t i = 0; i < N; ++i)
        {
            for(uint32_t j = 0; j < M; ++j)
            {
                ret.at(j,i) = at(i,j);
            }
        }
        return ret;
    }

    static Mat<N,M> eye() {
        Mat<N,M> ret(0);
        for(uint32_t i = 0; i < MIN(N,M); ++i) ret.at(i,i) = 1;
        return ret;
    }

};

template <uint32_t N>
using Square = Mat<N,N>;

template <uint32_t N>
using Vec = Mat<N,1>;

inline Vec<3> vec3(float x, float y, float z){
    Vec<3> ret;
    ret[0] = x;
    ret[1] = y;
    ret[2] = z;
    return ret;
}

inline Vec<2> vec2(float x, float y){
    Vec<2> ret;
    ret[0] = x;
    ret[1] = y;
    return ret;
}

template <uint32_t N, uint32_t M>
Mat<N,M> operator*(float val, const Mat<N,M>& m) {
    return m*val;
}


template <uint32_t N, uint32_t I, uint32_t M>
Mat<N,M> operator*(const Mat<N,I>& m1, const Mat<I,M>& m2)
{
    Mat<N,M> res;
    for(uint32_t r = 0; r < N; ++r){
        for(uint32_t c = 0; c < M; ++c){
            float sum = 0;
            for(uint32_t i = 0; i < I; ++i){
                sum += m1.at(r,i)*m2.at(i,c);
            }
            res.at(r,c) = sum;
        }
    }
    return res;
}

template <uint32_t N>
Square<N> covar_map(const Square<N>& m, const Square<N>& cov)
{
    Square<N> res;
    for(uint32_t r = 0; r < N; ++r){
        for(uint32_t c = r; c < N; ++c){
            float sum = 0;
            for(uint32_t i = 0; i < N; ++i){
                for(uint32_t j = 0; j < N; ++j){
                    sum += cov.at(i,j)*m.at(i,r)*m.at(j,c);
                }
            }
            res.at(r,c) = sum;
            res.at(c,r) = sum;
        }
    }
    return res;
}

inline float det(const Square<2>& m)
{
    return m.at(0,0)*m.at(1,1) - m.at(1,0)*m.at(0,1);
}

inline float det(const Square<3>& m)
{
    return
        m.at(0,0)*m.at(1,1)*m.at(2,2) + m.at(0,1)*m.at(1,2)*m.at(2,0) + m.at(0,2)*m.at(1,0)*m.at(2,1)
        -m.at(0,2)*m.at(1,1)*m.at(2,0) - m.at(0,1)*m.at(1,0)*m.at(2,2) - m.at(0,0)*m.at(1,2)*m.at(2,1);
}

inline Square<2> inv(const Square<2>& m)
{
    Square<2> ret;
    ret.at(0,0) = m.at(1,1);
    ret.at(1,1) = m.at(0,0);
    ret.at(1,0) = -m.at(1,0);
    ret.at(0,1) = -m.at(0,1);
    ret /= det(m);
    return ret;
}

template <uint32_t N>
float innerprod(const Vec<N>& v1, const Vec<N>& v2)
{
    float sum = 0;
    for(uint32_t i = 0; i < N; ++i)
    {
        sum += v1[i]*v2[i];
    }
    return sum;
}

inline Vec<3> crossp(const Vec<3>& v1, const Vec<3>& v2)
{
    return vec3(v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v1[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0]);
}

template <uint32_t N>
float normsquare(const Vec<N>& v)
{
    return innerprod(v,v);
}

template <uint32_t N>
float norm(const Vec<N>& v)
{
    return sqrtf(normsquare(v));
}

template <uint32_t N>
Square<N> outerprod(const Vec<N>& v1, const Vec<N>& v2)
{
    Square<N> ret;
    for(uint32_t r = 0; r < N; ++r){
        for(uint32_t c = 0; c < N; ++c){
            ret.at(r,c) = v1[r]*v2[c];
        }
    }
    return ret;
}

// Print functionality if building with iostream available (computer g++)
#ifndef ARDUINO

#include <iostream>
template <uint32_t N, uint32_t M>
std::ostream& operator<<(std::ostream& out, const Mat<N,M>& m)
{
    for(uint32_t i = 0; i < N; ++i){
        out << '[' << m.at(i,0);
        for(uint32_t j = 1; j < M; ++j)
        {
            out << ',' << m.at(i,j);
        }
        out << ']' << std::endl;
    }
    return out;
}

#endif



inline Square<3> inv(const Square<3>& m)
{
    Square<3> ret;
    ret.at(0, 0) = m.at(1, 1) * m.at(2, 2) - m.at(2, 1) * m.at(1, 2);
    ret.at(0, 1) = m.at(0, 2) * m.at(2, 1) - m.at(0, 1) * m.at(2, 2);
    ret.at(0, 2) = m.at(0, 1) * m.at(1, 2) - m.at(0, 2) * m.at(1, 1);
    ret.at(1, 0) = m.at(1, 2) * m.at(2, 0) - m.at(1, 0) * m.at(2, 2);
    ret.at(1, 1) = m.at(0, 0) * m.at(2, 2) - m.at(0, 2) * m.at(2, 0);
    ret.at(1, 2) = m.at(1, 0) * m.at(0, 2) - m.at(0, 0) * m.at(1, 2);
    ret.at(2, 0) = m.at(1, 0) * m.at(2, 1) - m.at(2, 0) * m.at(1, 1);
    ret.at(2, 1) = m.at(2, 0) * m.at(0, 1) - m.at(0, 0) * m.at(2, 1);
    ret.at(2, 2) = m.at(0, 0) * m.at(1, 1) - m.at(1, 0) * m.at(0, 1);
    ret /= det(m);
    return ret;
}

#endif
