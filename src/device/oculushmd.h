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

#ifndef OCULUSHMD_H
#define OCULUSHMD_H

#include <scenegraph/output/display/rendertotexturedisplay.h>
#include <scenegraph/input/singlebonetracker.h>
#include <OVR_CAPI.h>
#include <Kernel/OVR_Math.h>
#include <glm/gtc/quaternion.hpp>

namespace motorcar
{

//  As of SDKv0.3 Oculus has started to drop the C++ bindings in favor
//  of a CAPI as being the prefered stable way to interact with
//  the SDK. This wraps the OVR CAPI for us to use.

/**
 *  An OculusSDK system that generates devices. In the future Oculus
 *  will bring other devices of many types to the market.
 */

class OculusHMD;

class OVRSystem 
{
  public:
    OVRSystem();
    ~OVRSystem();

    /** Get a Device of type
     * HMD Display, tracker, Controller
     */
    //  @@todo should be able to pick which one; DKv1 or DKv2 or one of
    //  many attached DK's
    OculusHMD *getDisplay(OpenGLContext *glContext, Skeleton *skeleton, PhysicalNode *parent);
    //  @@todo Tracker independent of the display?
    //  @@todo Controller devices?

    /** @@todo  List Devices
     * Displays, Trackers, Controllers.
     */

    /** @@todo Handle Messages
     * Disconnects, Connects, User out of range
     * May have to integrate with dbus or something
     */
};

//  The Oculus HMD class
//  Contains: Display and Tracker
class OculusHMD : public RenderToTextureDisplay
{
  public:
    // Attempts to create an OculusHMD through the API, if something in the API Fails
    //(for example if no HMD's are present) this method returns a NULL pointer
    OculusHMD( ovrHmd hmd, OpenGLContext *glContext, Skeleton *skeleton, PhysicalNode *parent, float scale, glm::vec4 distortionK,
              glm::vec2 displayDimensions, const glm::mat4 &transform);
    ~OculusHMD();

    void prepareForDraw() override;

  private:
    /**  @@todo ovrHmdDesc will be depricated in v0.4, which will use the
     * ovrHmd object as the interface for ovrHmdDesc values.
     */
    ovrHmd m_hmd;
    ovrHmdDesc m_hmdDesc;

    float m_eyeHeight;

    //  @@todo Make a virtual tracker class, with position and base
    SingleBoneTracker *m_boneTracker;
};

} // namespace motorcar;

#endif // OCULUSHMD_H
