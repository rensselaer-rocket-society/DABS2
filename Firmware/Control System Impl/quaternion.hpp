#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdint.h>
#include <math.h>
#include <string.h>

class Quaternion;
float normsquare(const Quaternion& q);
float norm(const Quaternion& q);

class Quaternion {
public:
    float q[4];

    Quaternion() {};
    Quaternion(float r, float i, float j, float k){
        q[0] = r;
        q[1] = i;
        q[2] = j;
        q[3] = k;
    }
    Quaternion(float r, const Vec<3>& v) {
        q[0] = r;
        memcpy(q+1,v.d,3*sizeof(float));
    }

    inline float& operator[](int part){
        return q[part];
    }
    inline float operator[](int part) const {
        return q[part];
    }

    inline void operator+=(const Quaternion& q2) {
        for(uint32_t i = 0; i < 4; ++i) q[i] += q2[i];
    }
    Quaternion operator+(const Quaternion& q2) const {
        Quaternion ret(*this);
        ret += q2;
        return ret;
    }
    

    inline void operator-=(const Quaternion& q2) {
        for(uint32_t i = 0; i < 4; ++i) q[i] -= q2[i];
    }
    Quaternion operator-(const Quaternion& q2) const {
        Quaternion ret(*this);
        ret -= q2;
        return ret;
    }

    Quaternion operator-() const {
        Quaternion ret;
        for(uint32_t i = 0; i < 4; ++i) ret.q[i] = -q[i];
        return ret;
    }

    void operator*=(float a) {
        for(uint32_t i = 0; i < 4; ++i) q[i] *= a;
    }
    Quaternion operator*(float a) const {
        Quaternion ret(*this);
        ret *= a;
        return ret;
    }
    void operator/=(float a) {
        for(uint32_t i = 0; i < 4; ++i) q[i] /= a;
    }
    Quaternion operator/(float a) const {
        Quaternion ret(*this);
        ret /= a;
        return ret;
    }

    Quaternion operator*(const Quaternion& q2)
    {
        Quaternion ret;
        ret[0] = q[0]*q2[0] - q[1]*q2[1] - q[2]*q2[2] - q[3]*q2[3];
        ret[1] = q[0]*q2[1] + q[1]*q2[0] + q[2]*q2[3] - q[3]*q2[2];
        ret[2] = q[0]*q2[2] - q[1]*q2[3] + q[2]*q2[0] + q[3]*q2[1];
        ret[3] = q[0]*q2[3] + q[1]*q2[2] - q[2]*q2[1] + q[3]*q2[0];
        return ret;
    }

    void operator*=(const Quaternion& q2)
    {
        *this = (*this) * q2;
    }

    Quaternion conj() const {
        Quaternion ret;
        ret[0] = q[0];
        for(uint32_t i=1; i<4; ++i) ret[i] = -q[i];
        return ret;
    }

    inline Quaternion inv() const {
        return conj()/normsquare(*this);
    }

    inline void normalize() {
        *this /= norm(*this);
    }

    Vec<3> rotate(const Vec<3>& v) const {
        Quaternion vq(0,v);
        Quaternion r(*this);
        r *= vq;
        r *= conj();
        Vec<3> res;
        memcpy(res.d,r.q+1,3*sizeof(float));
        return res;
    }

     Vec<3> invrotate(const Vec<3>& v) const {
        Quaternion vq(0,v);
        Quaternion r = conj();
        r *= vq;
        r *= *this;
        Vec<3> res;
        memcpy(res.d,r.q+1,3*sizeof(float));
        return res;
    }
};

inline Quaternion operator*(float v, const Quaternion& q) {
    return q*v;
}

inline float normsquare(const Quaternion& q) {
    float sum = 0;
    for(uint32_t i = 0; i < 4; ++i) sum += q[i]*q[i];
    return sum;
}

inline float norm(const Quaternion& q) {
    return sqrtf(normsquare(q));
}

// Print functionality if building with iostream available (computer g++)
#ifndef ARDUINO

#include <iostream>

inline std::ostream& operator<<(std::ostream& out, const Quaternion& q)
{
    out << '[' << q[0] << ',' << '<'<< q[1] << ',' << q[2] << ',' << q[3] << '>' << ']' << std::endl;
    return out;
}

#else

#include <Print.h>

inline void printQuat(Print& out, const Quaternion& q)
{
    out.print('[');
    out.print(q[0]);
    for(uint32_t j = 1; j < 4; ++j)
    {
        out.print(',');
        out.print(q[j]);
    }
    out.println(']');
}

#endif


#endif