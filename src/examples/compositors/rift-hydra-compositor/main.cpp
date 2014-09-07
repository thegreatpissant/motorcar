/****************************************************************************
**
** Copyright (C) 2012 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the Qt Compositor.
**
** $QT_BEGIN_LICENSE:BSD$
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
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
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
** $QT_END_LICENSE$
**
****************************************************************************/

#include <motorcar.h>
#include <sixensemotionsensingsystem.h>
#include <oculushmd.h>

int main(int argc, char *argv[])
{
#ifdef GL_EXT_frag_depth
    std::cout << "depth extension available" << std::endl
#endif
        motorcar::Scene *scene = new motorcar::Scene();

    motorcar::Compositor *compositor = motorcar::Compositor::createCompositor(argc, argv, scene);
    scene->setCompositor(compositor);

    scene->setWindowManager(new motorcar::WindowManager(scene, compositor->defaultSeat()));

    motorcar::OpenGLContext *context = compositor->getContext();

    motorcar::Skeleton *skeleton = new motorcar::Skeleton(scene);
    motorcar::OVRSystem ovrSystem;

    motorcar::OculusHMD *hmd = ovrSystem.getDisplay(context, skeleton, scene);

    if (hmd) {
        std::cout << "Using Oculus Display" << std::endl;
        compositor->setDisplay(hmd);
    } else {
        std::cout << "Using Default Display" << std::endl;
        float camToDisplayDistance = 0.1f;
        motorcar::Display *display =
            new motorcar::Display(context, glm::vec2(0.325f, 0.1f), scene,
                                  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.8f, 1.25f)) *
                                      glm::rotate(glm::mat4(1.0f), -25.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
        display->addViewpoint(
            new motorcar::ViewPoint(0.01f, 100.0f, display, display,
                                    glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, camToDisplayDistance))));
        compositor->setDisplay(display);
    }

    scene->addDisplay(compositor->display());

    //    glm::mat4 cameraTransform = glm::rotate(glm::mat4(), 180.f, glm::vec3(0,1, 0)) * glm::scale(glm::mat4(),
    // glm::vec3(-1, 1, 1));
    //    motorcar::SoftKineticDepthCamera *ds325 = new motorcar::SoftKineticDepthCamera(scene, cameraTransform);

    motorcar::SixenseMotionSensingSystem *sixense = new motorcar::SixenseMotionSensingSystem(scene);
    if (sixense->isInitialized() && !sixense->baseStations().empty() &&
        !sixense->baseStations().front()->controllers().empty()) {

        motorcar::SixenseBaseNode *baseNode = sixense->baseStations().front();

        motorcar::SixenseControllerNode *headController = baseNode->controllers().front(),
                                        *handController = baseNode->controllers().back();

        // ds325->setParentNode(handController);
        // ds325->setParentNode(compositor->display());

        baseNode->setTransform(glm::translate(glm::mat4(1.0f), glm::vec3(0.5f, 0.25f, .25f)));

        handController->setPointingDevice(
            new motorcar::SixDOFPointingDevice(compositor->defaultSeat(), handController));

        headController->setBoneTracker(new motorcar::SingleBoneTracker(
            skeleton->headBone(), glm::translate(glm::mat4(), glm::vec3(0.0f, .073f, .184f)), skeleton, baseNode));

        std::cout << "parenting display to controller " << std::endl;
        compositor->display()->setParentNode(skeleton->headBone());

        glm::vec3 displayPosition = glm::vec3(0.0f, .127f, -.165f);

        glm::mat4 displayTransform = glm::translate(glm::mat4(), displayPosition);

        compositor->display()->setTransform(displayTransform);

        //        headController->setBoneTracker(NULL);
        //        compositor->display()->setParentNode(headController);
        //        compositor->display()->setTransform(glm::translate(glm::mat4(1), glm::vec3(0,-0.25, 0)));
    }

    std::cout << "Starting Compositor " << std::endl;

    int result = compositor->start();

    delete hmd;
    delete sixense;
    delete scene;

    return result;
}
