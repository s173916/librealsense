/* License: Apache 2.0. See LICENSE file in root directory.
Copyright(c) 2017 Intel Corporation. All Rights Reserved. */

#ifndef LIBREALSENSE_PYTHON_HPP
#define LIBREALSENSE_PYTHON_HPP

#include <pybind11/pybind11.h>

// convenience functions
#include <pybind11/operators.h>

// STL conversions
#include <pybind11/stl.h>

// std::chrono::*
#include <pybind11/chrono.h>

// makes certain STL containers opaque to prevent expensive copies
#include <pybind11/stl_bind.h>

// makes std::function conversions work
#include <pybind11/functional.h>

#define NAME pyrealsense2
#define SNAME "pyrealsense2"

// Hacky little bit of half-functions to make .def(BIND_DOWNCAST) look nice for binding as/is functions
#define BIND_DOWNCAST(class, downcast) "is_"#downcast, &rs2::class::is<rs2::downcast>).def("as_"#downcast, &rs2::class::as<rs2::downcast>

// Binding enums
const std::string rs2_prefix{ "rs2_" };
std::string make_pythonic_str(std::string str);
#define BIND_ENUM(module, rs2_enum_type, RS2_ENUM_COUNT, docstring)                                                         \
    static std::string rs2_enum_type##pyclass_name = std::string(#rs2_enum_type).substr(rs2_prefix.length());               \
    /* Above 'static' is required in order to keep the string alive since py::class_ does not copy it */                    \
    py::enum_<rs2_enum_type> py_##rs2_enum_type(module, rs2_enum_type##pyclass_name.c_str(), docstring);                    \
    /* std::cout << std::endl << "## " << rs2_enum_type##pyclass_name  << ":" << std::endl; */                              \
    for (int i = 0; i < static_cast<int>(RS2_ENUM_COUNT); i++)                                                              \
    {                                                                                                                       \
        rs2_enum_type v = static_cast<rs2_enum_type>(i);                                                                    \
        const char* enum_name = rs2_enum_type##_to_string(v);                                                               \
        auto python_name = make_pythonic_str(enum_name);                                                                    \
        py_##rs2_enum_type.value(python_name.c_str(), v);                                                                   \
        /* std::cout << " - " << python_name << std::endl;    */                                                            \
    }

// Binding arrays and matrices
template<typename T, size_t SIZE>
void copy_raw_array(T(&dst)[SIZE], const std::array<T, SIZE>& src)
{
    for (size_t i = 0; i < SIZE; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, size_t NROWS, size_t NCOLS>
void copy_raw_2d_array(T(&dst)[NROWS][NCOLS], const std::array<std::array<T, NCOLS>, NROWS>& src)
{
    for (size_t i = 0; i < NROWS; i++)
    {
        for (size_t j = 0; j < NCOLS; j++)
        {
            dst[i][j] = src[i][j];
        }
    }
}
template <typename T, size_t N>
std::string array_to_string(const T(&arr)[N])
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < N; i++)
    {
        if (i != 0)
            oss << ", ";
        oss << arr[i];
    }
    oss << "]";
    return oss.str();
}

template <typename T, size_t N, size_t M>
std::string matrix_to_string(const T(&arr)[N][M])
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < N; i++)
    {
        if (i != 0)
            oss << ", ";
        oss << "[";
        for (int j = 0; j < M; j++)
        {
            if (j != 0)
                oss << ", ";
            oss << arr[i][j];
        }
        oss << "]";
    }
    oss << "]";
    return oss.str();
}

#define BIND_RAW_ARRAY_GETTER(T, member, valueT, SIZE) [](const T& self) -> const std::array<valueT, SIZE>& { return reinterpret_cast<const std::array<valueT, SIZE>&>(self.member); }
#define BIND_RAW_ARRAY_SETTER(T, member, valueT, SIZE) [](T& self, const std::array<valueT, SIZE>& src) { copy_raw_array(self.member, src); }

#define BIND_RAW_2D_ARRAY_GETTER(T, member, valueT, NROWS, NCOLS) [](const T& self) -> const std::array<std::array<valueT, NCOLS>, NROWS>& { return reinterpret_cast<const std::array<std::array<valueT, NCOLS>, NROWS>&>(self.member); }
#define BIND_RAW_2D_ARRAY_SETTER(T, member, valueT, NROWS, NCOLS) [](T& self, const std::array<std::array<valueT, NCOLS>, NROWS>& src) { copy_raw_2d_array(self.member, src); }

#define BIND_RAW_ARRAY_PROPERTY(T, member, valueT, SIZE) #member, BIND_RAW_ARRAY_GETTER(T, member, valueT, SIZE), BIND_RAW_ARRAY_SETTER(T, member, valueT, SIZE)
#define BIND_RAW_2D_ARRAY_PROPERTY(T, member, valueT, NROWS, NCOLS) #member, BIND_RAW_2D_ARRAY_GETTER(T, member, valueT, NROWS, NCOLS), BIND_RAW_2D_ARRAY_SETTER(T, member, valueT, NROWS, NCOLS)

/*PYBIND11_MAKE_OPAQUE(std::vector<rs2::stream_profile>)*/

namespace py = pybind11;
using namespace pybind11::literals;

// Partial module definition functions
void init_c_files(py::module &m);
void init_types(py::module &m);
void init_frame(py::module &m);
void init_options(py::module &m);
void init_processing(py::module &m);
void init_sensor(py::module &m);
void init_device(py::module &m);
void init_record_playback(py::module &m);
void init_context(py::module &m);
void init_pipeline(py::module &m);
void init_internal(py::module &m);
void init_export(py::module &m);
void init_advanced_mode(py::module &m);
void init_util(py::module &m);

#endif // LIBREALSENSE_PYTHON_HPP
