#pragma once
#ifndef __ARRAY__
#define __ARRAY__

// namespace std {

template <class T, byte n> struct array {
    // Storage
    static const byte N = n;
    T data[N];

    static byte size() { return N; }
    using type = T;

    // Item access
    T &operator[](byte index) { return data[index]; }
    const T &operator[](byte index) const { return data[index]; }

    // Iterators
    T *begin() { return &data[0]; }
    const T *begin() const { return &data[0]; }
    T *end() { return &data[N]; }
    const T *end() const { return &data[N]; }

    // Comparisons
    bool operator==(const array<T, N> &rhs) const {
        if (this == &rhs)
            return true;
        for (byte i = 0; i < N; i++)
            if ((*this)[i] != rhs[i])
                return false;
        return true;
    }
    bool operator!=(const array<T, N> &rhs) const { return !(*this == rhs); }
};

// } // namespace std

#endif
