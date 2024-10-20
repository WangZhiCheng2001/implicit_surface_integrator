#pragma once

// forward declaration
struct Arrangement2D;
struct Arrangement3D;
template <size_t N>
struct IAComplex;

Arrangement2D extract_arrangement(IAComplex<2>&& ia_complex);
Arrangement3D extract_arrangement(IAComplex<3>&& ia_complex);