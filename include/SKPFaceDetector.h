#ifndef SKP_FACE_DETECTOR_H
#define SKP_FACE_DETECTOR_H

#include "SKPRecipient.h"
#include "SKWrapper.h"
#include <python3.6/Python.h>
#include <k4a/k4atypes.h>
#include <k4abt.hpp>
#include <vector>
#include <SKPacket.h>

const std::chrono::milliseconds TIMEOUT(K4A_WAIT_INFINITE);

class SKPFaceDetector : public SKPRecipient {
public:
    SKPFaceDetector(SKWrapper &skw);
    void receiveFrame(SKPacket &skp);
    void addRecipient(SKPRecipient *skpr);
    bool findTargetId();
    bool find3DTargetPose();
    bool chooseTarget();
    void getTargetEncoding();
    k4a_float3_t getTargetPosition();
    k4a_quaternion_t getTargetOrientation();

protected:
    std::vector<SKPRecipient*> _recipients;
    SKPacket buffer;
    PyObject* person_find;
    PyObject* get_encoding;
    PyObject* find_person;
    PyObject* target_encoding;
    int target_id;
    k4a_float3_t target_pos;
    k4a_quaternion_t target_orientation;
    k4abt::tracker tracker;
};

#endif