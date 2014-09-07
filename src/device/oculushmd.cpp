/****************************************************************************
**This file is part of the Motorcar 3D windowing framework
**
**
**Copyright (C) 2014 Forrest Reiling
**
**
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
**
****************************************************************************/
#include "oculushmd.h"

using namespace motorcar;

/**
 * OVRSystem
 */
OVRSystem::OVRSystem()
{
    //  @@todo make a singleton
    ovr_Initialize();
}
OVRSystem::~OVRSystem()
{
    // @@todo deallocate all devices
    ovr_Shutdown();
};

OculusHMD *OVRSystem::getDisplay(OpenGLContext *glContext, Skeleton *skeleton, PhysicalNode *parent)
{
    //  @@todo: Get requested device
    //  @@todo: if not found should we use a fake one? The SDK will let us.
    //  Assume at least one device at the moment
    ovrHmd hmd = ovrHmd_Create(0);

    if (!hmd) {
        std::cout << "Could not enumerate Oculus HMD Device" << std::endl;
        return nullptr;
    }

    //  @@todo This is going away in next SDK release
    ovrHmdDesc hmdDesc;
    ovrHmd_GetDesc(hmd, &hmdDesc);

    //  Now request our sensor Capabilities from the sensor
    if (false == ovrHmd_StartSensor(hmd, hmdDesc.SensorCaps, ovrSensorCap_Orientation))
        std::cerr << "Could not get required sensor Capabilities" << std::endl;

    //  Get HMD information
    //  @@todo SDKV0.4 this will change
    ovrHmd_GetDesc(hmd, &hmdDesc);

    //  Get the current defaults for EyeFov from SDK.  This will be the
    //  users defaults or the SDK defaults if not set.
    ovrFovPort eyeFov[ovrEye_Count];
    eyeFov[ovrEye_Left] = hmdDesc.DefaultEyeFov[ovrEye_Left];
    eyeFov[ovrEye_Right] = hmdDesc.DefaultEyeFov[ovrEye_Right];

    //  Get the SDK recomended client side distortion render values.
    ovrEyeRenderDesc eyeRenderDesc[ovrEye_Count];
    eyeRenderDesc[ovrEye_Left] = ovrHmd_GetRenderDesc(hmd, ovrEye_Left, eyeFov[ovrEye_Left]);
    eyeRenderDesc[ovrEye_Right] = ovrHmd_GetRenderDesc(hmd, ovrEye_Right, eyeFov[ovrEye_Right]);

    // @@todo HARD CODED DK1 VALUES pixel render is now mesh render
    int HResolution, VResolution;
    float HScreenSize, VScreenSize, VScreenCenter, EyeToScreenDistance, LensSeparationDistance, InterpupillaryDistance,
        scaleFactor;
    glm::vec4 DistortionK;

    HResolution = 1280;
    VResolution = 800;
    HScreenSize = .14976;
    VScreenSize = .0936;
    VScreenCenter = VScreenSize / 2;
    EyeToScreenDistance = 0.041f;
    LensSeparationDistance = .064;
    InterpupillaryDistance = .0647;
    DistortionK = glm::vec4(1, .22, .24, 0);
    scaleFactor = 1.25;

    float near = .01f, far = 10.0f;
    float h_meters = HScreenSize / 4.0f - LensSeparationDistance / 2.0f;
    float h = (4.0f * h_meters) / HScreenSize;

    OculusHMD * display = new OculusHMD(hmd, glContext, skeleton, parent, scaleFactor, DistortionK, glm::vec2(HScreenSize, VScreenSize),
                              glm::translate(glm::mat4(), glm::vec3(0.0f, 0.0f, EyeToScreenDistance)));

    ViewPoint *lCam =
        new ViewPoint(near, far, display, display,
                      glm::translate(glm::mat4(), glm::vec3(-InterpupillaryDistance / 2,
                                                            VScreenSize / 2 - VScreenCenter, EyeToScreenDistance)),
                      glm::vec4(0.0f, 0.0f, .5f, 1.0f), glm::vec3(h, 0.0f, 0.0f));

    ViewPoint *rCam =
        new ViewPoint(near, far, display, display,
                      glm::translate(glm::mat4(), glm::vec3(InterpupillaryDistance / 2.0f,
                                                            VScreenSize / 2 - VScreenCenter, EyeToScreenDistance)),
                      glm::vec4(.5f, 0.0f, .5f, 1.0f), glm::vec3(-h, 0.0f, 0.0f));

    display->addViewpoint(lCam);
    display->addViewpoint(rCam);

    return display;
    // OculusHMD(hmd, glContext, skeleton, parent);
}

#ifdef false
void nullfunction()
{
    // This will initialize HMDInfo with information about configured IPD,
    // screen size and other variables needed for correct projection.
    // We pass HMD DisplayDeviceName into the renderer to select the
    // correct monitor in full-screen mode.
    if (pHMD->GetDeviceInfo(&TheHMDInfo)) {
        // RenderParams.MonitorName = hmd.DisplayDeviceName;
        SConfig.SetHMDInfo(TheHMDInfo);
    }

    // Retrieve relevant profile settings.
    pUserProfile = pHMD->GetProfile();
    if (pUserProfile) {
        std::cout << "using profile with ipd: " << pUserProfile->GetIPD() << std::endl;
        m_eyeHeight = pUserProfile->GetEyeHeight();
    }
}

void OnMessage(const Message &msg)
{
    if (msg.Type == Message_DeviceAdded || msg.Type == Message_DeviceRemoved) {
        if (msg.pDevice == pManager) {
            const MessageDeviceStatus &statusMsg = static_cast<const MessageDeviceStatus &>(msg);

            { // limit the scope of the lock
                Lock::Locker lock(pManager->GetHandlerLock());
                // DeviceStatusNotificationsQueue.PushBack(
                // DeviceStatusNotificationDesc(statusMsg.Type, statusMsg.Handle));
            }

            switch (statusMsg.Type) {
            case OVR::Message_DeviceAdded:
                LogText("DeviceManager reported device added.\n");
                break;

            case OVR::Message_DeviceRemoved:
                LogText("DeviceManager reported device removed.\n");
                break;

            default:
                OVR_ASSERT(0); // unexpected type
            }
        }
    }
}


bool OVRSystem::SupportsMessageType(MessageType mt) const
{
    return true;
}
#endif

/**
 * OculusHMD
 */
OculusHMD::OculusHMD(ovrHmd hmd, OpenGLContext *glContext, Skeleton *skeleton, PhysicalNode *parent, float scale,
          glm::vec4 distortionK, glm::vec2 displayDimensions, const glm::mat4 &transform)
    : m_hmd(hmd), RenderToTextureDisplay(scale, distortionK, glContext, displayDimensions, parent, transform),
      m_boneTracker(new SingleBoneTracker(skeleton->headBone(), glm::mat4(), skeleton, parent))
{
}

OculusHMD::~OculusHMD()
{
}

void OculusHMD::prepareForDraw()
{
    
    ovrSensorState ss = ovrHmd_GetSensorState( m_hmd, 0.0 );
    ovrQuatf qf = ss.Predicted.Pose.Orientation;
    OVR::Quatf quatf( qf.x, qf.y, qf.z, qf.w); 

    OVR::Vector3f OVRaxis;
    float angle;
    quatf.GetAxisAngle(&OVRaxis, &angle);

    glm::vec3 axis = glm::vec3(OVRaxis.x, OVRaxis.y, OVRaxis.z);

    glm::quat orientation = glm::angleAxis(glm::degrees(angle), glm::normalize(axis));

    glm::vec3 parentPos = glm::vec3(parentNode()->worldTransform() * glm::vec4(0, 0, 0, 1));

    m_boneTracker->setOrientation(glm::mat3_cast(orientation));
}

