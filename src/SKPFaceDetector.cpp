#include "SKPFaceDetector.h"
#include "SKPacket.h"
#include <python3.6/Python.h>

#include <opencv2/opencv.hpp>
#include <numpy/arrayobject.h>
#include <iostream>
#include <limits>

using namespace std;

// #include <opencv2/opencv.hpp>

SKPFaceDetector::SKPFaceDetector(SKWrapper& skw) : _recipients(), buffer(&skw) {
    Py_Initialize();
    person_find = PyImport_ImportModule("person_find");
    cout << person_find << endl;
    get_encoding = PyObject_GetAttrString(person_find, "get_encoding");
    find_person = PyObject_GetAttrString(person_find, "find_person");
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker = k4abt::tracker::create(skw.getCalibration(), tracker_config);
    _import_array();
}

void SKPFaceDetector::getTargetEncoding() {
    cv::Mat &inMat = buffer.getCVMat("RGB1080p");
    buffer.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "face_detections");
    cv::Mat &faceMat = buffer.getCVMat("face_detections");

    cv::cvtColor(faceMat, faceMat, cv::COLOR_BGR2RGB);
    npy_intp dims[3] = {faceMat.rows, faceMat.cols, faceMat.channels()};
    PyObject* numpy_array = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, faceMat.data);
    target_encoding = PyObject_CallFunctionObjArgs(get_encoding, numpy_array, nullptr);
    
    Py_DECREF(numpy_array);

    // target_encoding = (double*) PyArray_DATA(target_encoding);
    // encoding_length = PyArray_DIMS(numpy_array)[0];

    // for (int i = 0; i < encoding_length; i++) {
    //     cout << target_encoding[i] << " ";
    // }
    // cout << endl;
}

bool SKPFaceDetector::findTargetId() {
    cout << "trying to find target id" << endl;
    cv::Mat &inMat = buffer.getCVMat("RGB1080p");
    buffer.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "face_detections");
    buffer.copyCVMat("RGB1080p", "face_detections");
    cv::Mat scene = buffer.getCVMat("face_detections");

    cv::cvtColor(scene, scene, cv::COLOR_BGR2RGB);
    npy_intp scene_dims[3] = {scene.rows, scene.cols, scene.channels()};
    PyObject* scene_numpy_array = PyArray_SimpleNewFromData(3, scene_dims, NPY_UINT8, scene.data);
    PyObject* found = PyObject_CallFunctionObjArgs(find_person, scene_numpy_array, target_encoding, nullptr);
    PyObject* first = PyTuple_GetItem(found, 0);
    PyObject* second = PyTuple_GetItem(found, 1);
    long x = PyLong_AS_LONG(first), y = PyLong_AS_LONG(second);

    Py_DECREF(scene_numpy_array);
    Py_DECREF(found);
    Py_DECREF(first);
    Py_DECREF(second);

    if (x == -1 && y == -1) {
        cout << "could not find target in frame" << endl;
        return false;
    }
    
    k4a::capture cap = buffer.getCapture();
    k4a::image depth_image = cap.get_depth_image();
    cv::Mat depth_mat(depth_image.get_height_pixels(), depth_image.get_width_pixels(), CV_16UC1, depth_image.get_buffer(), cv::Mat::AUTO_STEP);
    cv::Point2i point(x, y);
    k4a_float2_t pnt = {x, y};
    uint16_t depth_value = depth_mat.at<uint16_t>(point);

    k4a_float3_t target_3d;
    buffer.getSKWrapper()->getCalibration().convert_2d_to_3d(pnt, depth_value, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &target_3d);
    tracker.enqueue_capture(cap, TIMEOUT);

    k4abt::frame frame = tracker.pop_result(TIMEOUT);
    float minDist = std::numeric_limits<float>::max();
    int minIdx = -1;
    for (int i = 0; i < frame.get_num_bodies(); i++) {
        k4abt_skeleton_t curr = frame.get_body_skeleton(i);
        k4a_float3_t nose = curr.joints[K4ABT_JOINT_NOSE].position;
        float dx = target_3d.xyz.x - nose.xyz.x, dy = target_3d.xyz.y - nose.xyz.y, dz = target_3d.xyz.z - nose.xyz.z;
        float dist = dx * dx + dy * dy + dz * dz;
        if (dist < minDist) {
            minDist = dist;
            minIdx = i;
        }
    }
    target_id = frame.get_body_id(minIdx);
    cout << "id = " << target_id << endl;
    return true;
}

bool SKPFaceDetector::find3DTargetPose() {
    cout << "trying to find target 3d position" << endl;
    tracker.enqueue_capture(buffer.getCapture(), TIMEOUT);
    k4abt::frame frame = tracker.pop_result(TIMEOUT);
    int targetIdx = -1;
    for (int i = 0; i < frame.get_num_bodies(); i++) {
        if (frame.get_body_id(i) == target_id) {
            targetIdx = i;
            break;
        }
    }
    if (targetIdx == -1) {
        cout << "target is not in image" << endl;
        return false;
    }
    k4abt_joint_t target_joint = frame.get_body_skeleton(targetIdx).joints[K4ABT_JOINT_NOSE];
    target_pos = target_joint.position;
    target_orientation = target_joint.orientation;
    cout << "person is at (" << target_pos.xyz.x << ", " << target_pos.xyz.y << ", " << target_pos.xyz.z << ")" << endl;
    return true;
}

bool SKPFaceDetector::chooseTarget() {
    cout << "trying to choose target" << endl;
    cv::Mat &inMat = buffer.getCVMat("RGB1080p");
    buffer.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "face_detections");
    buffer.copyCVMat("RGB1080p", "face_detections");
    cv::Mat bgrMat = buffer.getCVMat("face_detections");
    cv::Mat faceMat;

    cv::cvtColor(bgrMat, faceMat, cv::COLOR_BGR2RGB);
    npy_intp dims[3] = {faceMat.rows, faceMat.cols, faceMat.channels()};
    PyObject* numpy_array = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, faceMat.data);
    target_encoding = PyObject_CallFunctionObjArgs(get_encoding, numpy_array, nullptr);
    int encoding_length = PyArray_DIM(target_encoding, 0);
    if (encoding_length == 0) {
        cout << "could not find any faces" << endl;
        return false;
    }
    cout << "found face, encoding = ";
    double* enc = (double*) PyArray_DATA(target_encoding);
    for (int i = 0; i < encoding_length; i++) {
        cout << enc[i] << " ";
    }
    cout << endl;
    return true;
}

k4a_float3_t SKPFaceDetector::getTargetPosition() {
    return target_pos;
}

k4a_quaternion_t SKPFaceDetector::getTargetOrientation() {
    return target_orientation;
}

void SKPFaceDetector::receiveFrame(SKPacket &skp) {
    buffer = skp;

    // if (!chose_target) {
    //     chooseTarget(skp);
    // } else if (!found_target) {
    //     findTargetId(skp);
    // } else {
    //     find3DTargetPose(skp);
    // }
    
    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

// void SKPFaceDetector::get3DPose(SKPacket& skp, double x, double y) {
//     SKWrapper wrapper = skp.getSKWrapper();
//     k4a::capture capture = skp.getCapture();
//     k4a::image color_image = capture.get_color_image();

//     cv::Mat color_mat(color_image.get_height_pixels, color_image.get_width_pixels,
//                         CV_8UC4, color_image.get_buffer, cv::Mat::AUTO_STEP);

//     // Convert the depth image to an OpenCV matrix
//     k4a::image depth_image = capture.get_depth_image();
//     cv::Mat depth_mat(depth_image.get_height_pixels, depth_image.get_width_pixels,
//                         CV_16UC1, depth_image.get_buffer, cv::Mat::AUTO_STEP);

//     // Convert the 2D point to a depth value
//     cv::Point2i point(x, y);
//     k4a_float2_t pnt = {x, y};
//     uint16_t depth_value = depth_mat.at<uint16_t>(point);

//     // Map the depth value to 3D world coordinates
//     k4a_float2_t point_3d;
//     k4a_calibration_t calibration;
//     wrapper.getCalibration().convert_2d_to_3d()
//     k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration);
//     k4a_calibration_2d_to_3d(&calibration, &(k4a_float2_t){ point.x, point.y }, depth_value, K4A_CALIBRATION_TYPE_DEPTH,
//                                 K4A_CALIBRATION_TYPE_DEPTH, &point_3d, NULL);
// }

void SKPFaceDetector::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);

}

/*
using namespace std;
// export PYTHONPATH=`pwd`
// g++ -g test.cpp -I/usr/include/python3.10 -I/home/krishagarwal/.local/lib/python3.10/site-packages/numpy/core/include/ -lpython3.10 -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc
int main(int argc, char const *argv[]) {
    Py_Initialize();
    import_array();
    PyObject* person_find = PyImport_ImportModule("person_find");
    PyObject* get_encoding = PyObject_GetAttrString(person_find, "get_encoding");
    PyObject* find_person = PyObject_GetAttrString(person_find, "find_person");
    cv::Mat target = cv::imread("leonardo.jpg");
    cv::cvtColor(target, target, cv::COLOR_BGR2RGB);
    npy_intp dims[3] = {target.rows, target.cols, target.channels()};
    PyObject* numpy_array = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, target.data);
    PyObject* target_encoding = PyObject_CallFunctionObjArgs(get_encoding, numpy_array, nullptr);
    cv::Mat scene = cv::imread("leonardo3.jpg");
    cv::cvtColor(scene, scene, cv::COLOR_BGR2RGB);
    npy_intp scene_dims[3] = {scene.rows, scene.cols, scene.channels()};
    PyObject* scene_numpy_array = PyArray_SimpleNewFromData(3, scene_dims, NPY_UINT8, scene.data);
    PyObject* found = PyObject_CallFunctionObjArgs(find_person, scene_numpy_array, target_encoding, nullptr);
    PyObject* first = PyTuple_GetItem(found, 0);
    PyObject* second = PyTuple_GetItem(found, 1);
    long x = PyLong_AS_LONG(first), y = PyLong_AS_LONG(second);
    cout << x << " " << y << endl;
    Py_DECREF(person_find);
    Py_DECREF(get_encoding);
    Py_DECREF(find_person);
    Py_DECREF(numpy_array);
    Py_DECREF(target_encoding);
}
*/