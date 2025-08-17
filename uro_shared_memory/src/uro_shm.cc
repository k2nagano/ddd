#include "uro_shared_memory/CameraImage.hh"
#include "uro_shared_memory/RobotName.hh"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace uro_shared_memory;

PYBIND11_MODULE(uro_shm, m)
{
    m.doc() = "uro_shm Python bindings";

    py::class_<CameraImage>(m, "CameraImage")
        .def(py::init<bool>(), py::arg("create") = false)
        .def(
            "SetImage",
            [](CameraImage& self, int width, int height, int channels,
               py::array_t<unsigned char> image) {
                py::buffer_info buf = image.request();

                // 期待されるサイズと一致しているか確認
                if (buf.size != width * height * channels)
                {
                    throw std::runtime_error("Image size does not match width * height * channels");
                }

                // ポインタを取得して SetImage を呼び出す
                unsigned char* ptr = static_cast<unsigned char*>(buf.ptr);
                return self.SetImage(width, height, channels, ptr);
            },
            py::arg("width"), py::arg("height"), py::arg("channels"), py::arg("image"))
        .def("GetImage",
             [](CameraImage& self) {
                 int width, height, channels;
                 std::vector<unsigned char> buffer;

                 // 仮に最大サイズを事前に確保（実際はサイズ取得後に再確保が望ましい）
                 buffer.resize(1920 * 1080 * 3); // 例: フルHD RGB

                 int result = self.GetImage(width, height, channels, buffer.data());
                 if (result != 0)
                     throw std::runtime_error("GetImage failed");

                 return py::make_tuple(
                     width, height, channels,
                     py::array_t<unsigned char>({height, width, channels}, buffer.data()));
             })
        .def("SetSerial", &CameraImage::SetSerial)
        .def("GetSerial",
             [](CameraImage& self) {
                 long serial;
                 self.GetSerial(serial);
                 return serial;
             })

        .def("SetRecording", &CameraImage::SetRecording)
        .def("GetRecording", [](CameraImage& self) {
            bool recording;
            self.GetRecording(recording);
            return recording;
        });

    py::class_<RobotName>(m, "RobotName")
        .def(py::init<bool>(), py::arg("master") = false)
        .def("Set", &RobotName::Set)
        .def("Get", [](RobotName& self) {
            std::string robot_name;
            self.Get(robot_name);
            return py::str(robot_name);
        });
}
